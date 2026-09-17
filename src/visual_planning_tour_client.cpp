/**
 * @file visual_planning_tour_client.cpp
 * @brief Client for the plan_visibility_tour service (global_plan step 4): plans
 *        ONE tour, draws it, and runs it on the robot.
 *
 * This is the single-run counterpart of src/vistsp_experiment_runner.cpp. The
 * runner answers "how do these methods compare over many problems" and executes
 * nothing; this answers "what does one run actually look like" and executes it.
 *
 * WHERE THE TARGETS COME FROM - two ways, exactly one of them per run:
 *   * ~sampling_regions_file: drawn at random from workspace regions, with
 *     ~num_targets and ~rng_seed. The draw is common/RunSetup.h's TargetSampler,
 *     the same object the experiment runner uses, so the SAME seed and count
 *     reproduce the experiment's first trial exactly - the run being watched is a
 *     member of the distribution being benchmarked, not a lookalike.
 *   * ~targets_yaml: fixed targets, in one of the three shapes below. Use this to
 *     re-run one specific problem.
 * Setting both is refused rather than silently preferring one.
 *
 * WHAT IT DOES WITH THE ANSWER: prints the per-rule comparison table, draws the
 * targets and the tour in RViz, and - unless ~execute is false - runs the
 * trajectory through PathExecuter, the same component the single-target client
 * uses. The robot is parked at ~start_state first, so execution starts where the
 * tour was planned from rather than jumping there.
 *
 * ---------------------------------------------------------------------------
 * WHY THE SERVICE TAKES SPHERES AND THIS FILE UNDERSTANDS CLOUDS
 * ---------------------------------------------------------------------------
 * A point cloud and a target sphere are NOT two alternative representations:
 * the sphere is DERIVED from the cloud, by enclosingBall() in common/Types.h.
 * Everything downstream of the service already speaks in spheres -- targets_ is
 * a std::vector<Ball>, the visibility oracle takes a centre and a radius -- so
 * the sphere is the canonical form and the cloud is an INPUT FORMAT.
 *
 * The conversion therefore happens HERE, at the input boundary, and
 * PlanVisibilityTour.srv carries spheres only. Two consequences worth keeping:
 *   - VisibilityTask.msg and PlanVisibilityPath.srv are untouched, so the
 *     single-target visibility planning path cannot break;
 *   - there is exactly one implementation of cloud-to-sphere, shared with
 *     VisibilityPlannerBase::computeTargetMES(), so the two can never disagree.
 *
 * Three YAML shapes are accepted, checked in this order:
 *
 *   targets:                        # spheres, given directly
 *     - center: [x, y, z]
 *       radius: r
 *
 *   target_clouds:                  # one point cloud PER TARGET
 *     - [[x,y,z], [x,y,z], ...]     # -> one sphere each
 *     - [[x,y,z], ...]
 *
 *   target_points:                  # legacy: one cloud for ONE target
 *     - [x, y, z]                   # -> a single sphere, so the existing
 *     - [x, y, z]                   #    config/targets.yaml still works
 */

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>

#include <cstdio>
#include <string>
#include <vector>

#include "visual_based_planning/PlanVisibilityTour.h"
#include "visual_based_planning/common/Types.h"
#include "visual_based_planning/common/RunSetup.h"
#include "visual_based_planning/components/PathExecuter.h"

namespace {

/// Reads a YAML [x, y, z] node.
bool readPoint(const YAML::Node& node, Eigen::Vector3d& out) {
    if (!node.IsSequence() || node.size() < 3) return false;
    out = Eigen::Vector3d(node[0].as<double>(),
                          node[1].as<double>(),
                          node[2].as<double>());
    return true;
}

using visual_planner::run_setup::TargetSphere;

/// Appends one ball to the target list. Targets are collected as TargetSphere
/// whichever way they were obtained, so the logging, the markers and the request
/// are built once rather than once per source.
void addTarget(std::vector<TargetSphere>& targets, const visual_planner::Ball& ball) {
    TargetSphere t;
    t.cx = ball.center.x();
    t.cy = ball.center.y();
    t.cz = ball.center.z();
    t.radius = ball.radius;
    t.region_id = -1;          ///< not drawn from a region
    targets.push_back(t);
}

/**
 * @brief Fills `targets` from the YAML, whichever of the three shapes it uses.
 * @return false if no recognised key held any usable target.
 */
bool loadTargets(const YAML::Node& config, std::vector<TargetSphere>& targets) {
    // --- shape 1: spheres, given directly ------------------------------
    if (config["targets"] && config["targets"].IsSequence()) {
        const YAML::Node& list = config["targets"];
        for (std::size_t i = 0; i < list.size(); ++i) {
            Eigen::Vector3d center;
            if (!list[i]["center"] || !readPoint(list[i]["center"], center)) {
                ROS_ERROR("targets[%lu] has no usable 'center: [x, y, z]'.", i);
                return false;
            }
            if (!list[i]["radius"]) {
                ROS_ERROR("targets[%lu] has no 'radius'. A target sphere needs "
                          "one; use 'target_clouds' if you want it derived "
                          "from points instead.", i);
                return false;
            }
            visual_planner::Ball ball;
            ball.center = center;
            ball.radius = list[i]["radius"].as<double>();
            addTarget(targets, ball);
        }
        ROS_INFO("Loaded %lu target sphere(s) from 'targets'.",
                 targets.size());
        return !targets.empty();
    }

    // --- shape 2: one point cloud per target ----------------------------
    if (config["target_clouds"] && config["target_clouds"].IsSequence()) {
        const YAML::Node& clouds = config["target_clouds"];
        for (std::size_t c = 0; c < clouds.size(); ++c) {
            std::vector<Eigen::Vector3d> points;
            for (std::size_t i = 0; i < clouds[c].size(); ++i) {
                Eigen::Vector3d p;
                if (!readPoint(clouds[c][i], p)) {
                    ROS_ERROR("target_clouds[%lu][%lu] is not [x, y, z].", c, i);
                    return false;
                }
                points.push_back(p);
            }
            if (points.empty()) {
                ROS_ERROR("target_clouds[%lu] is empty; a target needs at least "
                          "one point.", c);
                return false;
            }
            // The one and only cloud-to-sphere conversion (common/Types.h).
            addTarget(targets, visual_planner::enclosingBall(points));
        }
        ROS_INFO("Loaded %lu target(s) from 'target_clouds', each reduced to a "
                 "sphere by enclosingBall().", targets.size());
        return !targets.empty();
    }

    // --- shape 3: the legacy single-target cloud -------------------------
    if (config["target_points"] && config["target_points"].IsSequence()) {
        const YAML::Node& pts = config["target_points"];
        std::vector<Eigen::Vector3d> points;
        for (std::size_t i = 0; i < pts.size(); ++i) {
            Eigen::Vector3d p;
            if (!readPoint(pts[i], p)) {
                ROS_ERROR("target_points[%lu] is not [x, y, z].", i);
                return false;
            }
            points.push_back(p);
        }
        if (points.empty()) return false;

        addTarget(targets, visual_planner::enclosingBall(points));
        // Say this loudly: 'target_points' is the single-target format, so a
        // tour over it has exactly one stop and the GTSP part is trivial.
        // Anyone pointing this client at config/targets.yaml wants to know.
        ROS_WARN("Loaded 'target_points' (%lu points) as ONE target sphere -- "
                 "that is the single-target format. The tour will have a single "
                 "stop. Use 'target_clouds' or 'targets' to name several "
                 "targets.", points.size());
        return true;
    }

    ROS_ERROR("YAML has none of 'targets', 'target_clouds' or 'target_points'.");
    return false;
}

/// Prints the per-algorithm comparison the response carries.
void reportRuns(const visual_based_planning::PlanVisibilityTour::Response& res) {
    if (res.algorithm_names.empty()) {
        ROS_WARN("No algorithm results came back.");
        return;
    }
    ROS_INFO("--- tour costs by algorithm ---");
    ROS_INFO("%-12s %14s %14s %10s", "rule", "constructed", "after 2-opt", "gain");
    for (std::size_t i = 0; i < res.algorithm_names.size(); ++i) {
        const double before = (i < res.construction_costs.size())
                            ? res.construction_costs[i] : 0.0;
        const double after  = (i < res.improved_costs.size())
                            ? res.improved_costs[i] : 0.0;
        ROS_INFO("%-12s %14.4f %14.4f %9.2f%% %s",
                 res.algorithm_names[i].c_str(), before, after,
                 (before > 0.0) ? (100.0 * (before - after) / before) : 0.0,
                 (static_cast<int>(i) == res.best) ? "  <-- best" : "");
    }
}


/**
 * @brief Draws the tour: the end effector's path through the whole trajectory,
 *        and a labelled marker at every stop.
 *
 * WHY THE END EFFECTOR. The tour is a cycle in configuration space, which cannot
 * be drawn; the flashlight's path through the workspace is the readable proxy for
 * it, and it is also where the visibility happens. Stops carry the target index
 * they observe, which is the same index the target spheres are labelled with, so
 * "stop 2 watches target 0" can be read off the screen without the log.
 *
 * Decoration, so every failure here warns and returns: a run whose markers could
 * not be built is still a run.
 */
void drawTour(ros::Publisher& pub,
              const robot_model::RobotModelPtr& model,
              const std::string& group,
              const std::string& ee_link,
              const std::string& frame,
              const visual_based_planning::PlanVisibilityTour::Response& res) {
    const moveit::core::JointModelGroup* jmg = model->getJointModelGroup(group);
    if (!jmg) {
        ROS_WARN("Cannot draw the tour: no group '%s'.", group.c_str());
        return;
    }
    if (!model->hasLinkModel(ee_link)) {
        ROS_WARN("Cannot draw the tour: no link '%s'. Set ~ee_link to the "
                 "flashlight link if the node's planner/ee_link_name is not it.",
                 ee_link.c_str());
        return;
    }

    moveit::core::RobotState state(model);
    state.setToDefaultValues();

    std::vector<geometry_msgs::Point> path;
    path.reserve(res.trajectory.points.size());
    for (size_t i = 0; i < res.trajectory.points.size(); ++i) {
        state.setJointGroupPositions(jmg, res.trajectory.points[i].positions);
        state.update();
        const Eigen::Isometry3d& tf = state.getGlobalLinkTransform(ee_link);
        geometry_msgs::Point p;
        p.x = tf.translation().x();
        p.y = tf.translation().y();
        p.z = tf.translation().z();
        path.push_back(p);
    }
    if (path.empty()) return;

    visualization_msgs::MarkerArray array;

    visualization_msgs::Marker clear;
    clear.header.frame_id = frame;
    clear.header.stamp = ros::Time::now();
    clear.action = visualization_msgs::Marker::DELETEALL;
    array.markers.push_back(clear);

    visualization_msgs::Marker line;
    line.header.frame_id = frame;
    line.header.stamp = ros::Time::now();
    line.ns = "vistsp_tour_path";
    line.id = 0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.02;
    line.color.r = 0.1f; line.color.g = 0.8f; line.color.b = 0.9f; line.color.a = 0.9f;
    line.points = path;
    array.markers.push_back(line);

    for (size_t i = 0; i < res.stop_indices.size(); ++i) {
        const int idx = res.stop_indices[i];
        if (idx < 0 || idx >= static_cast<int>(path.size())) continue;
        const int target = (i < res.stop_target.size()) ? res.stop_target[i] : -1;

        visualization_msgs::Marker stop;
        stop.header.frame_id = frame;
        stop.header.stamp = ros::Time::now();
        stop.ns = "vistsp_tour_stops";
        stop.id = static_cast<int>(i);
        stop.type = visualization_msgs::Marker::SPHERE;
        stop.action = visualization_msgs::Marker::ADD;
        stop.pose.position = path[idx];
        stop.pose.orientation.w = 1.0;
        stop.scale.x = stop.scale.y = stop.scale.z = 0.08;
        // The start is its own colour: it is the only stop that observes nothing,
        // and the cycle both begins and ends there.
        if (target < 0) { stop.color.r = 1.0f; stop.color.g = 1.0f; stop.color.b = 1.0f; }
        else            { stop.color.r = 0.1f; stop.color.g = 0.9f; stop.color.b = 0.4f; }
        stop.color.a = 1.0f;
        array.markers.push_back(stop);

        visualization_msgs::Marker label = stop;
        label.ns = "vistsp_tour_labels";
        label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        label.pose.position.z += 0.12;
        label.scale.z = 0.12;
        label.color.r = label.color.g = label.color.b = 1.0f;
        label.text = (target < 0)
                   ? ("stop " + std::to_string(i) + ": start")
                   : ("stop " + std::to_string(i) + " -> target " + std::to_string(target));
        array.markers.push_back(label);
    }

    pub.publish(array);
    ROS_INFO("Drew the tour: %lu waypoints, %lu stops.", path.size(), res.stop_indices.size());
}
} // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "visual_planning_tour_client");
    ros::NodeHandle nh("~");
    ros::NodeHandle root;
    // MoveGroupInterface, inside PathExecuter, needs callbacks serviced while it
    // waits for the robot's state, so a spinner has to be running before it is
    // built - the single-target client does the same.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient client =
        root.serviceClient<visual_based_planning::PlanVisibilityTour>("/plan_visibility_tour");

    // --- what to solve ------------------------------------------------------
    std::string regions_file, targets_yaml;
    nh.param<std::string>("sampling_regions_file", regions_file, std::string());
    nh.param<std::string>("targets_yaml", targets_yaml, std::string());
    int num_targets = 5, rng_seed = 20260909, min_scene_objects = 1;
    double max_target_radius = 0.5, scene_wait_timeout = 120.0;
    nh.param("num_targets", num_targets, num_targets);
    nh.param("rng_seed", rng_seed, rng_seed);
    nh.param("max_target_radius", max_target_radius, max_target_radius);
    nh.param("min_scene_objects", min_scene_objects, min_scene_objects);
    nh.param("scene_wait_timeout", scene_wait_timeout, scene_wait_timeout);

    // Exactly one source. Refused rather than resolved by precedence: a launch
    // file that sets both means one of them is not doing what its author thinks.
    if (regions_file.empty() == targets_yaml.empty()) {
        ROS_FATAL("Set exactly one of ~sampling_regions_file (draw targets at random) "
                  "or ~targets_yaml (fixed targets). %s",
                  regions_file.empty() ? "Neither is set." : "Both are set.");
        return 1;
    }

    // --- how to start -------------------------------------------------------
    std::string start_state, planning_group, marker_frame, ee_link, planner_type;
    nh.param<std::string>("start_state", start_state, std::string("vertical"));
    nh.param<std::string>("planning_group", planning_group, std::string());
    nh.param<std::string>("marker_frame", marker_frame, std::string());
    nh.param<std::string>("ee_link", ee_link, std::string());
    nh.param<std::string>("planner_type", planner_type, std::string());
    bool park_robot = true, publish_markers = true, draw_tour = true, execute = true;
    nh.param("park_robot", park_robot, park_robot);
    nh.param("publish_markers", publish_markers, publish_markers);
    nh.param("draw_tour", draw_tour, draw_tour);
    nh.param("execute", execute, execute);
    double velocity_scaling = 1.0, acceleration_scaling = 1.0;
    nh.param("velocity_scaling", velocity_scaling, velocity_scaling);
    nh.param("acceleration_scaling", acceleration_scaling, acceleration_scaling);

    ros::Publisher target_pub =
        nh.advertise<visualization_msgs::MarkerArray>("vistsp_targets", 1, /*latch=*/true);
    ros::Publisher tour_pub =
        nh.advertise<visualization_msgs::MarkerArray>("vistsp_tour", 1, /*latch=*/true);

    ROS_INFO("Waiting for /plan_visibility_tour...");
    client.waitForExistence();

    // The model answers three things: the frame the markers belong in, the joints
    // a named start state names, and the forward kinematics the tour is drawn with.
    robot_model::RobotModelPtr model = visual_planner::run_setup::loadRobotModel();
    if (!model) return 1;
    if (marker_frame.empty()) marker_frame = model->getModelFrame();
    if (!visual_planner::run_setup::resolvePlanningGroup(root, planning_group)) return 1;
    if (ee_link.empty() &&
        !root.getParam("/visual_planning_node/planner/ee_link_name", ee_link)) {
        ROS_WARN("No ~ee_link and no /visual_planning_node/planner/ee_link_name; "
                 "the tour will not be drawn.");
    }

    std::vector<double> start_joints;
    std::vector<std::string> joint_names;
    if (!start_state.empty()) {
        if (!visual_planner::run_setup::resolveNamedStartState(
                model, planning_group, start_state, start_joints, joint_names)) {
            return 1;
        }
        if (park_robot) {
            visual_planner::run_setup::parkSimulatedRobot(root, joint_names, start_joints,
                                                          start_state);
        }
    } else {
        ROS_WARN("~start_state is empty: planning from the robot's CURRENT state, and "
                 "executing from wherever it happens to be.");
    }

    // --- the targets --------------------------------------------------------
    std::vector<TargetSphere> targets;
    if (!regions_file.empty()) {
        // Same order as the experiment runner: the scene first, because obstacles
        // decide which draws are rejected.
        if (!visual_planner::run_setup::waitForSceneObjects(min_scene_objects,
                                                            scene_wait_timeout)) {
            return 1;
        }
        std::vector<visual_planner::run_setup::AABB> regions, obstacles;
        if (!visual_planner::run_setup::loadRegions(regions_file, regions)) return 1;
        visual_planner::run_setup::collectSceneObstacles(obstacles);

        visual_planner::run_setup::TargetSampler sampler(
            regions, obstacles, static_cast<unsigned>(rng_seed), max_target_radius);
        for (int i = 0; i < num_targets; ++i) {
            TargetSphere t;
            if (!sampler.sample(t)) {
                ROS_FATAL("Gave up sampling target %d after 1000 attempts. Are the "
                          "sampling regions all inside obstacles?", i);
                return 1;
            }
            targets.push_back(t);
        }
        ROS_INFO("Drew %lu targets with seed %d - the same draw as trial 0 of an "
                 "experiment run with this seed, these regions and %lu targets per "
                 "trial.", targets.size(), rng_seed, targets.size());
    } else {
        YAML::Node config;
        try {
            config = YAML::LoadFile(targets_yaml);
        } catch (const std::exception& e) {
            ROS_FATAL("Failed to load YAML file '%s': %s", targets_yaml.c_str(), e.what());
            return 1;
        }
        if (!loadTargets(config, targets)) {
            ROS_FATAL("No usable targets in '%s'; refusing to call the service.",
                      targets_yaml.c_str());
            return 1;
        }
    }

    visual_planner::run_setup::logTargets(targets, marker_frame, "Run");
    if (publish_markers) {
        // Published BEFORE the call, which takes as long as the coverage query
        // does: the targets are on screen while the planner works on them.
        target_pub.publish(visual_planner::run_setup::targetMarkers(targets, marker_frame));
    }

    // --- ask ----------------------------------------------------------------
    visual_based_planning::PlanVisibilityTour srv;
    for (size_t i = 0; i < targets.size(); ++i) {
        visual_based_planning::VisibilityTarget t;
        t.center.x = targets[i].cx;
        t.center.y = targets[i].cy;
        t.center.z = targets[i].cz;
        t.radius   = targets[i].radius;
        srv.request.targets.push_back(t);
    }
    // The algorithm flags are NOT sent - they are planner/vistsp parameters on the
    // node, so that an unset field cannot silently disable an algorithm.
    srv.request.planner_type = planner_type;
    srv.request.start_joints = start_joints;

    ROS_INFO("Requesting a tour over %lu target(s)%s...", srv.request.targets.size(),
             planner_type.empty() ? "" : (" using " + planner_type).c_str());

    if (!client.call(srv)) {
        ROS_ERROR("Failed to call /plan_visibility_tour.");
        return 1;
    }

    reportRuns(srv.response);

    if (srv.response.coverage_incomplete) {
        // Not a failure: a tour needs only one reachable configuration per target,
        // so a coverage deficit still leaves a usable tour.
        ROS_WARN("The coverage query did not reach the requested k for every target. "
                 "The tour below is still valid.");
    }
    if (!srv.response.success) {
        ROS_ERROR("Tour planning failed.");
        return 1;
    }

    ROS_INFO("Tour found: %lu waypoints, %lu stops.",
             srv.response.trajectory.points.size(), srv.response.stop_indices.size());
    for (std::size_t i = 0; i < srv.response.stop_indices.size(); ++i) {
        const int target = (i < srv.response.stop_target.size())
                         ? srv.response.stop_target[i] : -1;
        if (target < 0) {
            ROS_INFO("  stop %lu at waypoint %d: the start configuration",
                     i, srv.response.stop_indices[i]);
        } else {
            ROS_INFO("  stop %lu at waypoint %d: observes target %d",
                     i, srv.response.stop_indices[i], target);
        }
    }

    // --- draw ---------------------------------------------------------------
    if (draw_tour && !ee_link.empty()) {
        drawTour(tour_pub, model, planning_group, ee_link, marker_frame, srv.response);
    }

    // --- run ----------------------------------------------------------------
    if (!execute) {
        ROS_INFO("~execute is false; the tour was planned and drawn but not run.");
        return 0;
    }

    std::vector<std::vector<double> > path;
    path.reserve(srv.response.trajectory.points.size());
    for (size_t i = 0; i < srv.response.trajectory.points.size(); ++i) {
        path.push_back(srv.response.trajectory.points[i].positions);
    }

    // The same component the single-target client executes with, on the group the
    // node planned for rather than a name repeated here. It time-parameterizes the
    // waypoints itself, which is why the tour can be handed over as positions.
    PathExecuter executer(planning_group);
    executer.setVelocityScaling(velocity_scaling);
    executer.setAccelerationScaling(acceleration_scaling);

    ROS_INFO("Executing the tour: %lu waypoints through %lu stops.",
             path.size(), srv.response.stop_indices.size());
    if (executer.executePath(path)) {
        ROS_INFO("Tour complete.");
        return 0;
    }
    ROS_ERROR("Execution failed.");
    return 1;
}
