/**
 * @file vistsp_experiment_runner.cpp
 * @brief Experiment driver for the VisTSP additions (global_plan steps 3-4).
 *
 * Samples a set of target spheres from the workspace regions, then evaluates
 * one or more tour METHODS on the same problem set, and reports timings and
 * tour costs.
 *
 * Modelled on src/experiment_runner.cpp, which serves the TVMP paper and is
 * left untouched. What is kept from it, and why:
 *   * PROBLEMS ARE PRE-GENERATED ONCE and reused by every method. That makes
 *     the comparison PAIRED -- every method sees the identical problem set --
 *     which is the only thing that makes the numbers comparable.
 *   * Region sampling, radius-from-region-bounds, and rejecting a sphere that
 *     intersects a scene obstacle.
 *   * Parameters set on the server between configurations, with a short settle.
 *
 * What differs:
 *   * A TRIAL IS A SET OF M TARGETS, not one. That is the whole point: a tour
 *     needs several targets to be a tour.
 *   * Regions are drawn WITH REPETITION within a trial, so two targets can land
 *     in one region and may overlap. That is wanted, not guarded against: an
 *     overlap is exactly the case where one configuration sees two targets, so
 *     it is what exercises node splitting (D3) and RP1's candidate sets. The
 *     per-trial region indices are recorded so those trials can be identified.
 *   * No point clouds are sampled. The tour service takes spheres directly, so
 *     the 100-points-per-sphere step of the old runner has nothing to do.
 *   * ONE CALL RETURNS EVERY RULE'S COST, before and after 2-opt. So there is
 *     deliberately no configuration per insertion rule and none for 2-opt
 *     on/off -- that would re-run the expensive coverage query to learn what a
 *     single call already reports. Configurations vary only what genuinely
 *     needs a separate run: the planner, the coverage k, and base locality.
 *
 * ---------------------------------------------------------------------------
 * ADDING GlobalVisTSP LATER
 * ---------------------------------------------------------------------------
 * TourProblem and TrialOutcome mention no method, and the statistics, printing
 * and file output touch only TrialOutcome. Exactly ONE function knows how to
 * invoke a specific method: callMethod(). So evaluating GlobalVisTSP is
 *   (1) one more MethodConfig row, and
 *   (2) one more branch in callMethod() for its service type.
 * Nothing in problem generation, statistics or reporting has to change, and the
 * two methods are automatically compared on the same problem set.
 */

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/CollisionObject.h>
#include <sensor_msgs/JointState.h>
#include <shape_msgs/SolidPrimitive.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "visual_based_planning/PlanVisibilityTour.h"

namespace {

// ---------------------------------------------------------------------------
// Problem description -- deliberately says nothing about how it will be solved
// ---------------------------------------------------------------------------

struct AABB {
    double min_x, max_x;
    double min_y, max_y;
    double min_z, max_z;
};

struct TargetSphere {
    double cx, cy, cz;
    double radius;
    int region_id = -1;   ///< which sampling region it came from
};

/// One trial: a set of targets to be toured. Generated once, solved by every
/// method under test.
struct TourProblem {
    int id = -1;
    std::vector<TargetSphere> targets;
    /// True when two of this trial's targets overlap, which is the case that
    /// exercises node splitting. Recorded so those trials can be separated out.
    bool has_overlap = false;
};

// ---------------------------------------------------------------------------
// Method under test
// ---------------------------------------------------------------------------

/// How a tour method is invoked. LocalVisTSP is the only kind implemented;
/// GlobalVisTSP will add another `kind` and a branch in callMethod().
enum class MethodKind {
    LOCAL_VISTSP     ///< the plan_visibility_tour service
    // , GLOBAL_VISTSP  <-- add here when global_plan steps 5-6 land
};

struct MethodConfig {
    std::string name;              ///< appears in the report and the CSV
    MethodKind  kind = MethodKind::LOCAL_VISTSP;
    std::string planner_mode;      ///< "VisRRT" or "VisPRM"
    int         coverage = 1;      ///< configurations required per target
    /// <= 0 means the base is unconstrained, i.e. the whole environment is
    /// toured. That is the intended baseline, not a missing setting.
    double      base_locality = 0.0;
    bool        use_nearest = true;
    bool        use_farthest = true;
    bool        use_cheapest = true;
    bool        use_two_opt = true;
};

// ---------------------------------------------------------------------------
// Result of one trial -- also says nothing about which method produced it
// ---------------------------------------------------------------------------

struct TrialOutcome {
    int  problem_id = -1;
    bool call_ok = false;         ///< the service call itself went through
    bool success = false;         ///< a tour came back
    bool coverage_incomplete = false;

    double wall_seconds = 0.0;    ///< measured by this process, round trip
    double coverage_seconds = 0.0;
    double instance_seconds = 0.0;
    double gtsp_seconds = 0.0;
    double expand_seconds = 0.0;
    double server_seconds = 0.0;  ///< the server's own total

    double tour_cost = std::numeric_limits<double>::quiet_NaN();
    std::string winning_rule;
    int num_stops = 0;
    int num_waypoints = 0;

    /// Parallel to `rule_names`; NaN where a rule did not run.
    std::vector<std::string> rule_names;
    std::vector<double> rule_construction;
    std::vector<double> rule_improved;
};

struct Stats {
    double avg = 0.0, median = 0.0, min = 0.0, max = 0.0, std_dev = 0.0;
    int count = 0;
};

Stats computeStats(const std::vector<double>& data) {
    Stats s;
    std::vector<double> clean;
    for (size_t i = 0; i < data.size(); ++i) {
        if (!std::isnan(data[i])) clean.push_back(data[i]);
    }
    s.count = static_cast<int>(clean.size());
    if (clean.empty()) return s;

    const double sum = std::accumulate(clean.begin(), clean.end(), 0.0);
    s.avg = sum / clean.size();
    std::sort(clean.begin(), clean.end());
    s.min = clean.front();
    s.max = clean.back();
    s.median = (clean.size() % 2 == 0)
             ? (clean[clean.size() / 2 - 1] + clean[clean.size() / 2]) / 2.0
             : clean[clean.size() / 2];
    double sq = 0.0;
    for (size_t i = 0; i < clean.size(); ++i) {
        sq += (clean[i] - s.avg) * (clean[i] - s.avg);
    }
    s.std_dev = std::sqrt(sq / clean.size());
    return s;
}

/// Squared-distance test, as in the old runner.
bool sphereIntersectsAABB(double cx, double cy, double cz, double r,
                          const AABB& box) {
    double sq = 0.0;
    if (cx < box.min_x) sq += (box.min_x - cx) * (box.min_x - cx);
    if (cx > box.max_x) sq += (cx - box.max_x) * (cx - box.max_x);
    if (cy < box.min_y) sq += (box.min_y - cy) * (box.min_y - cy);
    if (cy > box.max_y) sq += (cy - box.max_y) * (cy - box.max_y);
    if (cz < box.min_z) sq += (box.min_z - cz) * (box.min_z - cz);
    if (cz > box.max_z) sq += (cz - box.max_z) * (cz - box.max_z);
    return sq <= r * r;
}

} // namespace


class VisTSPExperimentRunner {
public:
    explicit VisTSPExperimentRunner(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh) {
        tour_client_ =
            nh_.serviceClient<visual_based_planning::PlanVisibilityTour>(
                "plan_visibility_tour");

        pnh_.param("num_trials", num_trials_, 20);
        pnh_.param("targets_per_trial", targets_per_trial_, 5);
        pnh_.param<std::string>("sampling_regions_file", regions_file_,
            "/home/roblab22/catkin_ws/src/visual_based_planning/config/"
            "sampling_regions_double_room.yaml");
        pnh_.param("publish_markers", publish_markers_, true);
        pnh_.param<std::string>("marker_frame", marker_frame_, std::string());
        pnh_.param<std::string>("results_prefix", results_prefix_,
                                std::string("vistsp_experiments"));

        // --- start configuration -------------------------------------------
        // A NAMED SRDF STATE, not a list of joint values: the group is built
        // from subgroups (mobile_base + manipulator), so a hand-written vector
        // would silently depend on the variable order MoveIt happens to use.
        // Resolving the name through the robot model uses the same order the
        // node's copyJointGroupPositions() does, by construction.
        pnh_.param<std::string>("start_state", start_state_name_,
                                std::string("vertical"));
        // Empty means "ask the node for the group the planner is configured
        // with". The node loads planner_config.yaml into its own private
        // namespace, so that is where the one authoritative value lives, and
        // duplicating it here would let the two drift apart.
        pnh_.param<std::string>("planning_group", planning_group_, std::string());
        pnh_.param("park_robot", park_robot_, true);

        // --- scene -----------------------------------------------------------
        // The experiment is meaningless in an empty room: with no obstacles
        // there is no occlusion, so every target is trivially visible. Waiting
        // for the scene, and refusing to run without it, is what keeps a
        // forgotten load_scene.launch from producing plausible-looking numbers.
        pnh_.param("min_scene_objects", min_scene_objects_, 1);
        pnh_.param("scene_wait_timeout", scene_wait_timeout_, 120.0);

        // --- target size -----------------------------------------------------
        // The radius rule below is "distance to the nearest region bound", so a
        // region's shape alone decides how big its targets are: the 0.2 m wall
        // bands of the double room give ~0.1, the 1 x 1 x 2 cells of the pillar
        // grid give up to 0.5. This is the ceiling on that. The default is 0.5,
        // which changes nothing for either environment - it exists so target size
        // can be turned into an experimental variable without reshaping regions.
        pnh_.param("max_target_radius", max_target_radius_, 0.5);
        if (max_target_radius_ <= 0.0) {
            ROS_FATAL("~max_target_radius is %.3f; it must be positive. A zero or "
                      "negative cap rejects every sphere and the run would report "
                      "'no targets' rather than a bad setting.", max_target_radius_);
            throw std::runtime_error("max_target_radius must be positive");
        }
        // A FIXED default seed so a problem set is reproducible: the whole
        // point of pre-generating is that every method sees the same problems,
        // and that should hold across invocations too, not just within one.
        // Change it deliberately to get a different problem set.
        int seed = 0;
        pnh_.param("rng_seed", seed, 20260909);
        rng_ = std::mt19937(static_cast<unsigned>(seed));
        ROS_INFO("VisTSP experiments: %d trials x %d targets, seed %d.",
                 num_trials_, targets_per_trial_, seed);

        if (publish_markers_) {
            marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>(
                "vistsp_targets", 1, /*latch=*/true);
        }
        if (park_robot_) {
            // The topic demo.launch's joint_state_publisher merges into
            // /joint_states (its source_list). Publishing here moves the
            // SIMULATED robot; on hardware nothing subscribes and parking is
            // skipped with a warning.
            park_pub_ = nh_.advertise<sensor_msgs::JointState>(
                "/move_group/fake_controller_joint_states", 1);
        }
    }

    void run() {
        ROS_INFO("Waiting for plan_visibility_tour...");
        tour_client_.waitForExistence();

        // The robot model is loaded ONCE and shared: the marker frame and the
        // start configuration both come from it, and two loads could disagree
        // if robot_description were reloaded in between.
        if (!loadRobotModel()) return;
        if (!resolveStartState()) return;
        parkRobot();

        if (!loadRegions()) return;
        // Before the scene is read, not after: obstacles decide which sampled
        // targets are rejected, so problems generated against an empty world
        // would not be the problems the planner then solves.
        if (!waitForScene()) return;
        loadObstacles();
        if (!generateProblems()) return;

        // The methods to evaluate. Note what is NOT here: no row per insertion
        // rule and no 2-opt on/off row, because one call reports every rule's
        // cost before and after 2-opt. Rows differ only where a separate
        // coverage query is genuinely required.
        std::vector<MethodConfig> configs;
        {
            MethodConfig c;
            c.name = "LocalVisTSP-global-VisRRT";
            c.planner_mode = "VisRRT";
            c.coverage = 1;
            c.base_locality = 0.0;   // the whole environment: the baseline
            configs.push_back(c);

            // c.name = "LocalVisTSP-global-VisPRM";
            // c.planner_mode = "VisPRM";
            // configs.push_back(c);

            // c.name = "LocalVisTSP-global-VisRRT-k3";
            // c.planner_mode = "VisRRT";
            // c.coverage = 3;          // more choices per target for the tour
            // configs.push_back(c);
        }

        std::vector<std::string> names;
        std::vector<std::vector<TrialOutcome> > per_method;
        for (size_t m = 0; m < configs.size(); ++m) {
            ROS_INFO("------------------------------------------------");
            ROS_INFO("Method: %s", configs[m].name.c_str());
            names.push_back(configs[m].name);
            per_method.push_back(runMethod(configs[m]));
            report(configs[m].name, per_method.back());
        }

        saveText(names, per_method, configs);
        saveCsv(names, per_method);
        ROS_INFO("Done. Wrote %s.txt and %s.csv",
                 results_prefix_.c_str(), results_prefix_.c_str());
    }

private:
    // -----------------------------------------------------------------------
    // Problem generation
    // -----------------------------------------------------------------------

    bool loadRegions() {
        try {
            YAML::Node config = YAML::LoadFile(regions_file_);
            for (const auto& node : config["sampling_regions"]) {
                AABB r;
                r.min_x = node["min"][0].as<double>();
                r.min_y = node["min"][1].as<double>();
                r.min_z = node["min"][2].as<double>();
                r.max_x = node["max"][0].as<double>();
                r.max_y = node["max"][1].as<double>();
                r.max_z = node["max"][2].as<double>();
                regions_.push_back(r);
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to load sampling regions from '%s': %s",
                      regions_file_.c_str(), e.what());
            return false;
        }
        if (regions_.empty()) {
            ROS_ERROR("No sampling regions in '%s'.", regions_file_.c_str());
            return false;
        }
        ROS_INFO("Loaded %lu sampling regions.", regions_.size());
        return true;
    }

    /**
     * @brief Loads robot_description once, for the marker frame and the start state.
     *
     * @return false when the model will not load. That is fatal rather than a
     *         warning: without it neither the markers nor the start
     *         configuration can be placed, and running anyway would produce an
     *         experiment whose start is whatever the robot happened to be doing.
     */
    bool loadRobotModel() {
        robot_model_loader::RobotModelLoader loader("robot_description");
        robot_model_ = loader.getModel();
        if (!robot_model_) {
            ROS_FATAL("Could not load 'robot_description'. Is MoveIt running? "
                      "The experiments need mobile_ur_moveit_config demo.launch "
                      "(or the robot's bring-up) started first.");
            return false;
        }
        if (marker_frame_.empty()) {
            // No frame convention exists anywhere in this repo, so ask the
            // robot model rather than hardcoding one that happens to work on
            // one robot. The model frame IS the planning frame, and it is what
            // the collision objects are expressed in.
            // (PlanningSceneInterface has no getPlanningFrame() in Melodic.)
            marker_frame_ = robot_model_->getModelFrame();
            ROS_INFO("Marker frame taken from the robot model: '%s'.",
                     marker_frame_.c_str());
        }
        return true;
    }

    /**
     * @brief Resolves the planning group the start state belongs to.
     *
     * Order: the ~planning_group parameter, then the node's own
     * planner/group_name. The node is the authority -- it is the process that
     * actually plans -- so its value is preferred over a default repeated here.
     *
     * @return false when neither is set, which is a refusal rather than a guess:
     *         a wrong group would resolve "vertical" against the wrong joints.
     */
    bool resolvePlanningGroup() {
        if (!planning_group_.empty()) return true;
        if (nh_.getParam("/visual_planning_node/planner/group_name", planning_group_)
            && !planning_group_.empty()) {
            ROS_INFO("Planning group taken from the node: '%s'.",
                     planning_group_.c_str());
            return true;
        }
        ROS_FATAL("No planning group: the node's "
                  "/visual_planning_node/planner/group_name is unset and no "
                  "~planning_group was given. Refusing to guess which joints "
                  "'%s' names.", start_state_name_.c_str());
        return false;
    }

    /**
     * @brief Resolves ~start_state into the joint vector EVERY trial starts from.
     *
     * WHY A FIXED START. The tour is a cycle through the start configuration,
     * so the start is part of the problem, not of the setup. Leaving it at
     * "whatever the robot is doing" makes two runs of the same seed
     * incomparable, which would defeat the pre-generated problem set.
     *
     * The name is looked up among the group's SRDF group_states. An unknown
     * name is FATAL and lists what exists -- falling back to the current state
     * would hide a typo behind numbers that still look reasonable.
     *
     * An explicitly empty ~start_state keeps the old behaviour (the node uses
     * the robot's current state) and says so loudly.
     *
     * @return false when the run must not proceed.
     */
    bool resolveStartState() {
        if (start_state_name_.empty()) {
            ROS_WARN("~start_state is empty: every trial starts from the robot's "
                     "CURRENT state, whatever it is when the trial runs. Two runs "
                     "of the same seed are then only comparable if nothing moved "
                     "the robot in between.");
            return true;                       // start_joints_ stays empty
        }
        if (!resolvePlanningGroup()) return false;

        const moveit::core::JointModelGroup* jmg =
            robot_model_->getJointModelGroup(planning_group_);
        if (!jmg) {
            ROS_FATAL("Group '%s' does not exist in the robot model.",
                      planning_group_.c_str());
            return false;
        }

        moveit::core::RobotState state(robot_model_);
        state.setToDefaultValues();
        if (!state.setToDefaultValues(jmg, start_state_name_)) {
            std::string known;
            const std::vector<std::string>& names = jmg->getDefaultStateNames();
            for (size_t i = 0; i < names.size(); ++i) {
                known += (i ? ", " : "") + names[i];
            }
            ROS_FATAL("Group '%s' has no state named '%s'. Known states: [%s]. "
                      "Refusing to run: falling back to the current state would "
                      "quietly change what the experiment measures.",
                      planning_group_.c_str(), start_state_name_.c_str(),
                      known.c_str());
            return false;
        }
        state.update();
        state.copyJointGroupPositions(jmg, start_joints_);
        // Same order as the positions above, which is what makes the parking
        // message below consistent with what is sent to the planner.
        park_joint_names_ = jmg->getVariableNames();

        std::string joints;
        for (size_t i = 0; i < start_joints_.size(); ++i) {
            char buf[64];
            snprintf(buf, sizeof(buf), "%s%.3f", i ? ", " : "", start_joints_[i]);
            joints += buf;
        }
        ROS_INFO("Start state '%s' of group '%s' resolved to [%s].",
                 start_state_name_.c_str(), planning_group_.c_str(), joints.c_str());
        return true;
    }

    /**
     * @brief Moves the SIMULATED robot to the resolved start state.
     *
     * Cosmetic for the planner -- the start is sent explicitly with every
     * request -- but not cosmetic for the scene: the context snapshots the
     * planning scene on the first request, and that snapshot carries the
     * current state, which seeds collision checking for joints outside the
     * planning group. Parking first keeps that snapshot identical across runs.
     *
     * Simulation only. On hardware nothing subscribes to the fake controller
     * topic, and a robot cannot teleport anyway, so this warns and gives up.
     */
    void parkRobot() {
        if (!park_robot_ || start_joints_.empty()) return;

        sensor_msgs::JointState js;
        js.name = park_joint_names_;
        js.position = start_joints_;

        // The subscriber is another node that may still be starting, and a
        // message published before it connects is simply lost.
        const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(5.0);
        while (ros::ok() && park_pub_.getNumSubscribers() == 0 &&
               ros::WallTime::now() < deadline) {
            ros::Duration(0.1).sleep();
        }
        if (park_pub_.getNumSubscribers() == 0) {
            ROS_WARN("Nothing subscribes to /move_group/fake_controller_joint_states, "
                     "so the robot was NOT parked at '%s'. Harmless on hardware and "
                     "without demo.launch: the tour requests still carry the start "
                     "configuration explicitly.", start_state_name_.c_str());
            return;
        }
        // Repeated because the joint_state_publisher merges sources on its own
        // timer; one message can land between two of its publications.
        for (int i = 0; i < 10 && ros::ok(); ++i) {
            js.header.stamp = ros::Time::now();
            park_pub_.publish(js);
            ros::Duration(0.1).sleep();
        }
        ROS_INFO("Parked the simulated robot at '%s'.", start_state_name_.c_str());
    }

    /**
     * @brief Blocks until the planning scene holds at least ~min_scene_objects.
     *
     * THE ORDERING THIS SOLVES. The scene has to be applied after the planner
     * node's scene monitor is subscribed, so it cannot simply be loaded first;
     * but the runner reads it immediately after the service appears. Waiting
     * here removes the race in both directions, and a timeout is a refusal
     * rather than a warning because an empty world produces an experiment with
     * no occlusion at all -- numbers that look fine and mean nothing.
     *
     * @return false when the scene never arrived.
     */
    bool waitForScene() {
        if (min_scene_objects_ <= 0) {
            ROS_WARN("~min_scene_objects <= 0: not waiting for a scene. The "
                     "experiment will run in whatever world is loaded, including "
                     "an empty one.");
            return true;
        }
        moveit::planning_interface::PlanningSceneInterface psi;
        const ros::WallTime deadline =
            ros::WallTime::now() + ros::WallDuration(scene_wait_timeout_);
        ros::WallTime next_log = ros::WallTime::now();
        while (ros::ok()) {
            const std::vector<std::string> known = psi.getKnownObjectNames();
            if (static_cast<int>(known.size()) >= min_scene_objects_) {
                ROS_INFO("Planning scene holds %lu objects.", known.size());
                return true;
            }
            if (ros::WallTime::now() > deadline) {
                ROS_FATAL("Waited %.0f s and the planning scene still holds %lu "
                          "objects (< %d). Load the environment -- e.g. "
                          "'roslaunch scene_builder load_scene.launch' -- and note "
                          "it must be applied AFTER visual_planning_node starts.",
                          scene_wait_timeout_, known.size(), min_scene_objects_);
                return false;
            }
            if (ros::WallTime::now() >= next_log) {
                ROS_INFO("Waiting for the planning scene (%lu/%d objects)...",
                         known.size(), min_scene_objects_);
                next_log = ros::WallTime::now() + ros::WallDuration(5.0);
            }
            ros::Duration(0.5).sleep();
        }
        return false;
    }

    /// Scene obstacles as AABBs, so a sampled target can be rejected if it is
    /// buried in one. Same approach as the old runner.
    void loadObstacles() {
        moveit::planning_interface::PlanningSceneInterface psi;
        std::map<std::string, moveit_msgs::CollisionObject> objects = psi.getObjects();
        for (const auto& kv : objects) {
            const auto& obj = kv.second;
            for (size_t i = 0; i < obj.primitives.size(); ++i) {
                if (obj.primitives[i].type != shape_msgs::SolidPrimitive::BOX) continue;
                if (i >= obj.primitive_poses.size()) continue;
                const double dx = obj.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_X];
                const double dy = obj.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
                const double dz = obj.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Z];
                const auto& p = obj.primitive_poses[i].position;
                AABB o;
                o.min_x = p.x - dx / 2.0; o.max_x = p.x + dx / 2.0;
                o.min_y = p.y - dy / 2.0; o.max_y = p.y + dy / 2.0;
                o.min_z = p.z - dz / 2.0; o.max_z = p.z + dz / 2.0;
                obstacles_.push_back(o);
            }
        }
        ROS_INFO("Extracted %lu box obstacles from the planning scene.",
                 obstacles_.size());
    }

    /**
     * @brief One target: pick a region, sample a centre, take the largest
     *        radius that stays inside the region, reject if it hits an obstacle.
     *
     * The radius rule is the old runner's: the shortest distance from the
     * centre to the region's bounds. For the thin slab regions of
     * sampling_regions_double_room.yaml that caps the radius near 0.1 m, which
     * is what keeps a target inside the wall band it was sampled from.
     */
    bool sampleTarget(TargetSphere& out) {
        std::uniform_int_distribution<int> pick(0, static_cast<int>(regions_.size()) - 1);
        for (int attempt = 0; attempt < 1000; ++attempt) {
            const int region_id = pick(rng_);
            const AABB& reg = regions_[region_id];

            std::uniform_real_distribution<double> dx(reg.min_x, reg.max_x);
            std::uniform_real_distribution<double> dy(reg.min_y, reg.max_y);
            std::uniform_real_distribution<double> dz(reg.min_z, reg.max_z);
            TargetSphere t;
            t.region_id = region_id;
            t.cx = dx(rng_);
            t.cy = dy(rng_);
            t.cz = dz(rng_);
            t.radius = std::min(std::min(std::min(t.cx - reg.min_x, reg.max_x - t.cx),
                                         std::min(t.cy - reg.min_y, reg.max_y - t.cy)),
                                std::min(t.cz - reg.min_z, reg.max_z - t.cz));
            // Capped AFTER the bounds rule, never instead of it: the bounds are
            // what keeps a sphere inside the band it was drawn from, and the cap
            // only makes it smaller.
            t.radius = std::min(t.radius, max_target_radius_);
            if (t.radius <= 0.0) continue;

            bool hit = false;
            for (size_t o = 0; o < obstacles_.size() && !hit; ++o) {
                if (sphereIntersectsAABB(t.cx, t.cy, t.cz, t.radius, obstacles_[o])) {
                    hit = true;
                }
            }
            if (hit) continue;

            out = t;
            return true;
        }
        return false;
    }

    bool generateProblems() {
        problems_.clear();
        for (int i = 0; i < num_trials_; ++i) {
            TourProblem prob;
            prob.id = i;
            for (int j = 0; j < targets_per_trial_; ++j) {
                TargetSphere t;
                if (!sampleTarget(t)) {
                    ROS_ERROR("Gave up sampling target %d of trial %d after 1000 "
                              "attempts. Are the sampling regions all inside "
                              "obstacles?", j, i);
                    return false;
                }
                prob.targets.push_back(t);
            }
            // Regions are drawn WITH REPETITION, so two targets of one trial can
            // overlap. Flag it rather than prevent it: an overlap means one
            // configuration can see two targets, which is what node splitting
            // exists for, so those trials are the interesting ones.
            for (size_t a = 0; a < prob.targets.size() && !prob.has_overlap; ++a) {
                for (size_t b = a + 1; b < prob.targets.size(); ++b) {
                    const TargetSphere& u = prob.targets[a];
                    const TargetSphere& v = prob.targets[b];
                    const double dx = u.cx - v.cx, dy = u.cy - v.cy, dz = u.cz - v.cz;
                    if (std::sqrt(dx*dx + dy*dy + dz*dz) < u.radius + v.radius) {
                        prob.has_overlap = true;
                        break;
                    }
                }
            }
            problems_.push_back(prob);
        }
        int overlapping = 0;
        for (size_t i = 0; i < problems_.size(); ++i) {
            if (problems_[i].has_overlap) ++overlapping;
        }
        ROS_INFO("Generated %lu trials of %d targets (%d with overlapping "
                 "targets, which exercise node splitting).",
                 problems_.size(), targets_per_trial_, overlapping);
        return true;
    }

    // -----------------------------------------------------------------------
    // Marker publishing
    // -----------------------------------------------------------------------

    /**
     * @brief Publishes one trial's targets as RViz markers.
     *
     * A translucent SPHERE at each target's true radius, plus a text label with
     * the target INDEX -- the labels matter because the response's stop_target
     * refers to those indices, so without them a tour cannot be read off the
     * screen. Latched, so RViz picks them up whenever it subscribes.
     */
    void publishTargets(const TourProblem& prob) {
        // Printed even when markers are off, and printed HERE rather than at
        // generation time, so the log lists exactly what was drawn for the trial
        // being solved. The planner prints the same three numbers per target from
        // beginCoverageRun(); if the two lists differ, what is on screen is not the
        // problem being solved. Indices are shared with the marker labels and with
        // the tour response's stop_target.
        ROS_INFO("Trial %d targets (frame %s)%s:", prob.id,
                 marker_frame_.empty() ? "<unset>" : marker_frame_.c_str(),
                 prob.has_overlap ? ", CONTAINS OVERLAPPING TARGETS" : "");
        for (size_t i = 0; i < prob.targets.size(); ++i) {
            const TargetSphere& t = prob.targets[i];
            ROS_INFO("  target %lu: center (%.4f, %.4f, %.4f), radius %.4f  [region %d]",
                     i, t.cx, t.cy, t.cz, t.radius, t.region_id);
        }

        if (!publish_markers_) return;

        visualization_msgs::MarkerArray array;
        // A DELETEALL first, so the previous trial's markers do not linger and
        // get mistaken for this trial's problem.
        visualization_msgs::Marker clear;
        clear.header.frame_id = marker_frame_;
        clear.header.stamp = ros::Time::now();
        clear.action = visualization_msgs::Marker::DELETEALL;
        array.markers.push_back(clear);

        for (size_t i = 0; i < prob.targets.size(); ++i) {
            const TargetSphere& t = prob.targets[i];

            visualization_msgs::Marker m;
            m.header.frame_id = marker_frame_;
            m.header.stamp = ros::Time::now();
            m.ns = "vistsp_targets";
            m.id = static_cast<int>(i);
            m.type = visualization_msgs::Marker::SPHERE;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = t.cx;
            m.pose.position.y = t.cy;
            m.pose.position.z = t.cz;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = 2.0 * t.radius;  // diameter
            m.color.r = 1.0f; m.color.g = 0.55f; m.color.b = 0.0f;
            m.color.a = 0.6f;
            array.markers.push_back(m);

            visualization_msgs::Marker label = m;
            label.ns = "vistsp_target_labels";
            label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            label.pose.position.z = t.cz + t.radius + 0.05;
            label.scale.z = 0.12;
            label.color.r = label.color.g = label.color.b = 1.0f;
            label.color.a = 1.0f;
            label.text = std::to_string(i);
            array.markers.push_back(label);
        }
        marker_pub_.publish(array);
    }

    // -----------------------------------------------------------------------
    // Invocation -- THE ONLY METHOD-SPECIFIC CODE IN THIS FILE
    // -----------------------------------------------------------------------

    /**
     * @brief Applies a method's parameters to the node. Called once per method.
     */
    void applyMethodParams(const MethodConfig& cfg) {
        nh_.setParam("/planner/mode", cfg.planner_mode);
        nh_.setParam("/planner/vistsp/coverage", cfg.coverage);
        nh_.setParam("/planner/vistsp/base_locality_radius", cfg.base_locality);
        nh_.setParam("/planner/vistsp/use_nearest_insertion", cfg.use_nearest);
        nh_.setParam("/planner/vistsp/use_farthest_insertion", cfg.use_farthest);
        nh_.setParam("/planner/vistsp/use_cheapest_insertion", cfg.use_cheapest);
        nh_.setParam("/planner/vistsp/use_two_opt", cfg.use_two_opt);
        // The node reads planner/vistsp from its PRIVATE namespace, so set both
        // and let whichever the node resolves win. Setting only the global one
        // would silently leave the file's defaults in place.
        const std::string p = "/visual_planning_node/planner/vistsp/";
        nh_.setParam(p + "coverage", cfg.coverage);
        nh_.setParam(p + "base_locality_radius", cfg.base_locality);
        nh_.setParam(p + "use_nearest_insertion", cfg.use_nearest);
        nh_.setParam(p + "use_farthest_insertion", cfg.use_farthest);
        nh_.setParam(p + "use_cheapest_insertion", cfg.use_cheapest);
        nh_.setParam(p + "use_two_opt", cfg.use_two_opt);
        ros::Duration(0.5).sleep();   // let the node observe the change
    }

    /**
     * @brief Runs one method on one problem.
     *
     * ADDING GlobalVisTSP: give MethodKind a new value and branch here. Nothing
     * else in this file needs to know, because TrialOutcome is method-agnostic.
     */
    TrialOutcome callMethod(const MethodConfig& cfg, const TourProblem& prob) {
        TrialOutcome out;
        out.problem_id = prob.id;

        if (cfg.kind != MethodKind::LOCAL_VISTSP) {
            ROS_ERROR("Method '%s' names a kind this runner cannot invoke yet.",
                      cfg.name.c_str());
            return out;
        }

        visual_based_planning::PlanVisibilityTour srv;
        srv.request.planner_type = cfg.planner_mode;
        for (size_t i = 0; i < prob.targets.size(); ++i) {
            visual_based_planning::VisibilityTarget t;
            t.center.x = prob.targets[i].cx;
            t.center.y = prob.targets[i].cy;
            t.center.z = prob.targets[i].cz;
            t.radius   = prob.targets[i].radius;
            srv.request.targets.push_back(t);
        }
        // The resolved start state, so every method and every trial starts
        // from the identical configuration. Left empty only when ~start_state
        // was explicitly cleared, in which case the node falls back to the
        // robot's current state.
        srv.request.start_joints = start_joints_;

        const ros::WallTime t0 = ros::WallTime::now();
        out.call_ok = tour_client_.call(srv);
        out.wall_seconds = (ros::WallTime::now() - t0).toSec();
        if (!out.call_ok) return out;

        const auto& r = srv.response;
        out.success = r.success;
        out.coverage_incomplete = r.coverage_incomplete;
        out.coverage_seconds = r.coverage_seconds;
        out.instance_seconds = r.instance_seconds;
        out.gtsp_seconds = r.gtsp_seconds;
        out.expand_seconds = r.expand_seconds;
        out.server_seconds = r.total_seconds;
        out.num_stops = static_cast<int>(r.stop_indices.size());
        out.num_waypoints = static_cast<int>(r.trajectory.points.size());

        for (size_t i = 0; i < r.algorithm_names.size(); ++i) {
            out.rule_names.push_back(r.algorithm_names[i]);
            out.rule_construction.push_back(
                i < r.construction_costs.size() ? r.construction_costs[i]
                    : std::numeric_limits<double>::quiet_NaN());
            out.rule_improved.push_back(
                i < r.improved_costs.size() ? r.improved_costs[i]
                    : std::numeric_limits<double>::quiet_NaN());
        }
        if (r.best >= 0 && r.best < static_cast<int>(out.rule_names.size())) {
            out.winning_rule = out.rule_names[r.best];
            out.tour_cost = out.rule_improved[r.best];
        }
        return out;
    }

    std::vector<TrialOutcome> runMethod(const MethodConfig& cfg) {
        applyMethodParams(cfg);
        std::vector<TrialOutcome> outcomes;
        for (size_t i = 0; i < problems_.size(); ++i) {
            publishTargets(problems_[i]);
            outcomes.push_back(callMethod(cfg, problems_[i]));
            if (!outcomes.back().call_ok) {
                ROS_WARN("  trial %lu: service call failed.", i);
            }
            if (i % 5 == 0) {
                ROS_INFO("  progress %lu/%lu", i, problems_.size());
            }
        }
        return outcomes;
    }

    // -----------------------------------------------------------------------
    // Reporting -- touches TrialOutcome only, so it is method-agnostic
    // -----------------------------------------------------------------------

    static std::vector<double> column(const std::vector<TrialOutcome>& o,
                                      double TrialOutcome::*field,
                                      bool successes_only) {
        std::vector<double> v;
        for (size_t i = 0; i < o.size(); ++i) {
            if (successes_only && !o[i].success) continue;
            v.push_back(o[i].*field);
        }
        return v;
    }

    void report(const std::string& name, const std::vector<TrialOutcome>& o) {
        int ok = 0, incomplete = 0;
        for (size_t i = 0; i < o.size(); ++i) {
            if (o[i].success) ++ok;
            if (o[i].coverage_incomplete) ++incomplete;
        }
        const Stats total = computeStats(column(o, &TrialOutcome::server_seconds, true));
        const Stats cov   = computeStats(column(o, &TrialOutcome::coverage_seconds, true));
        const Stats inst  = computeStats(column(o, &TrialOutcome::instance_seconds, true));
        const Stats gtsp  = computeStats(column(o, &TrialOutcome::gtsp_seconds, true));
        const Stats exp   = computeStats(column(o, &TrialOutcome::expand_seconds, true));
        const Stats cost  = computeStats(column(o, &TrialOutcome::tour_cost, true));

        std::cout << "\nResults for " << name << ":\n";
        std::cout << "  success " << ok << "/" << o.size()
                  << ", coverage incomplete on " << incomplete << "\n";
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "  -- time, seconds (successful trials) --\n";
        std::cout << "    coverage query : avg " << cov.avg   << "  med " << cov.median  << "\n";
        std::cout << "    instance/closure: avg " << inst.avg << "  med " << inst.median << "\n";
        std::cout << "    gtsp solve     : avg " << gtsp.avg  << "  med " << gtsp.median << "\n";
        std::cout << "    expand+smooth  : avg " << exp.avg   << "  med " << exp.median  << "\n";
        std::cout << "    server total   : avg " << total.avg << "  med " << total.median << "\n";
        std::cout << std::setprecision(4);
        std::cout << "  -- tour cost (successful trials) --\n";
        std::cout << "    avg " << cost.avg << "  med " << cost.median
                  << "  min " << cost.min << "  max " << cost.max << "\n";

        // Per-rule table, averaged over trials where that rule produced a tour.
        printRuleTable(o);
        std::cout << std::endl;
    }

    void printRuleTable(const std::vector<TrialOutcome>& o) {
        std::vector<std::string> names;
        for (size_t i = 0; i < o.size(); ++i) {
            for (size_t r = 0; r < o[i].rule_names.size(); ++r) {
                if (std::find(names.begin(), names.end(), o[i].rule_names[r])
                    == names.end()) {
                    names.push_back(o[i].rule_names[r]);
                }
            }
        }
        if (names.empty()) return;

        std::cout << "  -- cost by rule (avg over successful trials) --\n";
        std::cout << "    " << std::left << std::setw(12) << "rule"
                  << std::right << std::setw(14) << "constructed"
                  << std::setw(14) << "after 2-opt"
                  << std::setw(10) << "gain" << std::setw(8) << "wins" << "\n";
        for (size_t n = 0; n < names.size(); ++n) {
            std::vector<double> before, after;
            int wins = 0;
            for (size_t i = 0; i < o.size(); ++i) {
                if (!o[i].success) continue;
                if (o[i].winning_rule == names[n]) ++wins;
                for (size_t r = 0; r < o[i].rule_names.size(); ++r) {
                    if (o[i].rule_names[r] != names[n]) continue;
                    before.push_back(o[i].rule_construction[r]);
                    after.push_back(o[i].rule_improved[r]);
                }
            }
            const Stats b = computeStats(before), a = computeStats(after);
            const double gain = (b.avg > 0.0) ? 100.0 * (b.avg - a.avg) / b.avg : 0.0;
            std::cout << "    " << std::left << std::setw(12) << names[n]
                      << std::right << std::fixed << std::setprecision(4)
                      << std::setw(14) << b.avg << std::setw(14) << a.avg
                      << std::setw(9) << std::setprecision(2) << gain << "%"
                      << std::setw(8) << wins << "\n";
        }
    }

    void saveText(const std::vector<std::string>& names,
                  const std::vector<std::vector<TrialOutcome> >& per_method,
                  const std::vector<MethodConfig>& configs) {
        std::ofstream f(results_prefix_ + ".txt");
        if (!f.is_open()) {
            ROS_ERROR("Cannot open %s.txt for writing.", results_prefix_.c_str());
            return;
        }
        f << "VisTSP Experiment Results\n=========================\n\n";
        f << "trials: " << problems_.size()
          << ", targets per trial: " << targets_per_trial_
          << ", regions file: " << regions_file_ << "\n";
        f << "max target radius: " << max_target_radius_ << " m\n";
        f << "start state: "
          << (start_joints_.empty() ? std::string("<robot's current state>")
                                    : start_state_name_ + " (group " + planning_group_ + ")")
          << ", scene objects: " << obstacles_.size() << " boxes\n";
        f << "Regions are drawn WITH REPETITION, so a trial may contain "
             "overlapping targets;\nthose are the trials that exercise node "
             "splitting. Per-trial detail is in the CSV.\n\n";

        for (size_t m = 0; m < names.size(); ++m) {
            const auto& o = per_method[m];
            const auto& c = configs[m];
            f << "Method: " << names[m] << "\n";
            f << "  planner " << c.planner_mode << ", coverage k " << c.coverage
              << ", base locality " << c.base_locality
              << (c.base_locality > 0.0 ? " (local)" : " (global / baseline)")
              << "\n";
            int ok = 0;
            for (size_t i = 0; i < o.size(); ++i) if (o[i].success) ++ok;
            f << "  successes " << ok << " / " << o.size() << "\n";
            const char* labels[] = {"coverage", "instance", "gtsp", "expand", "total"};
            double TrialOutcome::*fields[] = {
                &TrialOutcome::coverage_seconds, &TrialOutcome::instance_seconds,
                &TrialOutcome::gtsp_seconds, &TrialOutcome::expand_seconds,
                &TrialOutcome::server_seconds};
            for (int k = 0; k < 5; ++k) {
                const Stats s = computeStats(column(o, fields[k], true));
                f << "  time " << labels[k] << ": avg " << s.avg
                  << " med " << s.median << " min " << s.min
                  << " max " << s.max << " sd " << s.std_dev << "\n";
            }
            const Stats cost = computeStats(column(o, &TrialOutcome::tour_cost, true));
            f << "  tour cost: avg " << cost.avg << " med " << cost.median
              << " min " << cost.min << " max " << cost.max
              << " sd " << cost.std_dev << "\n\n";
        }
        f.close();
    }

    /// One row per (method, trial). This is the file to plot from.
    void saveCsv(const std::vector<std::string>& names,
                 const std::vector<std::vector<TrialOutcome> >& per_method) {
        std::ofstream f(results_prefix_ + ".csv");
        if (!f.is_open()) {
            ROS_ERROR("Cannot open %s.csv for writing.", results_prefix_.c_str());
            return;
        }
        f << "method,trial,num_targets,has_overlap,success,coverage_incomplete,"
             "wall_s,coverage_s,instance_s,gtsp_s,expand_s,server_total_s,"
             "tour_cost,winning_rule,num_stops,num_waypoints,"
             "rule,rule_constructed,rule_improved\n";
        for (size_t m = 0; m < names.size(); ++m) {
            for (size_t i = 0; i < per_method[m].size(); ++i) {
                const TrialOutcome& o = per_method[m][i];
                const TourProblem& p = problems_[i];
                // One line per rule, so the file is tidy for grouping. Trials
                // with no rule data still get a line, so failures are visible.
                const size_t rules = std::max<size_t>(1, o.rule_names.size());
                for (size_t r = 0; r < rules; ++r) {
                    f << names[m] << ',' << o.problem_id << ','
                      << p.targets.size() << ',' << (p.has_overlap ? 1 : 0) << ','
                      << (o.success ? 1 : 0) << ','
                      << (o.coverage_incomplete ? 1 : 0) << ','
                      << o.wall_seconds << ',' << o.coverage_seconds << ','
                      << o.instance_seconds << ',' << o.gtsp_seconds << ','
                      << o.expand_seconds << ',' << o.server_seconds << ','
                      << o.tour_cost << ',' << o.winning_rule << ','
                      << o.num_stops << ',' << o.num_waypoints << ',';
                    if (r < o.rule_names.size()) {
                        f << o.rule_names[r] << ',' << o.rule_construction[r]
                          << ',' << o.rule_improved[r];
                    } else {
                        f << ",,";
                    }
                    f << '\n';
                }
            }
        }
        f.close();
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::ServiceClient tour_client_;
    ros::Publisher marker_pub_;
    ros::Publisher park_pub_;
    std::mt19937 rng_;

    int num_trials_ = 20;
    int targets_per_trial_ = 5;
    bool publish_markers_ = true;
    std::string regions_file_;
    std::string marker_frame_;
    std::string results_prefix_;

    robot_model::RobotModelPtr robot_model_;
    std::string start_state_name_;
    std::string planning_group_;
    /// Empty only when ~start_state was cleared: then the node uses the
    /// robot's current state, as it did before this was configurable.
    std::vector<double> start_joints_;
    std::vector<std::string> park_joint_names_;
    bool park_robot_ = true;
    int min_scene_objects_ = 1;
    double scene_wait_timeout_ = 120.0;
    double max_target_radius_ = 0.5;

    std::vector<AABB> regions_;
    std::vector<AABB> obstacles_;
    std::vector<TourProblem> problems_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vistsp_experiment_runner");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    VisTSPExperimentRunner runner(nh, pnh);
    runner.run();
    return 0;
}
