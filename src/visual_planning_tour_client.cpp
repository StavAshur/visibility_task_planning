/**
 * @file visual_planning_tour_client.cpp
 * @brief Client for the plan_visibility_tour service (global_plan step 4).
 *
 * Reads the targets from a YAML file named by the `targets_yaml` private
 * parameter -- the same pattern visual_ik_client.cpp uses -- calls the tour
 * service, and prints the per-algorithm comparison table the response carries.
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

namespace {

/// Reads a YAML [x, y, z] node.
bool readPoint(const YAML::Node& node, Eigen::Vector3d& out) {
    if (!node.IsSequence() || node.size() < 3) return false;
    out = Eigen::Vector3d(node[0].as<double>(),
                          node[1].as<double>(),
                          node[2].as<double>());
    return true;
}

/// Appends one target ball to the request.
void addTarget(visual_based_planning::PlanVisibilityTour& srv,
               const visual_planner::Ball& ball) {
    visual_based_planning::VisibilityTarget t;
    t.center.x = ball.center.x();
    t.center.y = ball.center.y();
    t.center.z = ball.center.z();
    t.radius = ball.radius;
    srv.request.targets.push_back(t);
}

/**
 * @brief Fills srv.request.targets from the YAML, whichever shape it uses.
 * @return false if no recognised key held any usable target.
 */
bool loadTargets(const YAML::Node& config,
                 visual_based_planning::PlanVisibilityTour& srv) {
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
            addTarget(srv, ball);
        }
        ROS_INFO("Loaded %lu target sphere(s) from 'targets'.",
                 srv.request.targets.size());
        return !srv.request.targets.empty();
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
            addTarget(srv, visual_planner::enclosingBall(points));
        }
        ROS_INFO("Loaded %lu target(s) from 'target_clouds', each reduced to a "
                 "sphere by enclosingBall().", srv.request.targets.size());
        return !srv.request.targets.empty();
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

        addTarget(srv, visual_planner::enclosingBall(points));
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

} // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "visual_planning_tour_client");
    ros::NodeHandle nh("~");

    ros::ServiceClient client =
        nh.serviceClient<visual_based_planning::PlanVisibilityTour>(
            "/plan_visibility_tour");

    ROS_INFO("Waiting for /plan_visibility_tour...");
    client.waitForExistence();

    std::string yaml_path;
    if (!nh.getParam("targets_yaml", yaml_path)) {
        ROS_ERROR("Missing param '~targets_yaml'. Set it in the launch file.");
        return 1;
    }

    YAML::Node config;
    try {
        config = YAML::LoadFile(yaml_path);
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to load YAML file '%s': %s", yaml_path.c_str(), e.what());
        return 1;
    }

    visual_based_planning::PlanVisibilityTour srv;
    if (!loadTargets(config, srv)) {
        ROS_ERROR("No usable targets in '%s'; refusing to call the service.",
                  yaml_path.c_str());
        return 1;
    }

    // Optional: the mode. Empty leaves the node's planner/mode in charge. The
    // algorithm flags are NOT sent -- they are planner/vistsp parameters on the
    // node, so that an unset field cannot silently disable an algorithm.
    std::string planner_type;
    nh.param<std::string>("planner_type", planner_type, std::string());
    srv.request.planner_type = planner_type;

    // Optional explicit start; empty means "use the robot's current state".
    std::vector<double> start_joints;
    if (nh.getParam("start_joints", start_joints)) {
        srv.request.start_joints = start_joints;
        ROS_INFO("Using an explicit start configuration (%lu joints).",
                 start_joints.size());
    }

    ROS_INFO("Requesting a tour over %lu target(s)%s...",
             srv.request.targets.size(),
             planner_type.empty() ? "" : (" using " + planner_type).c_str());

    if (!client.call(srv)) {
        ROS_ERROR("Failed to call /plan_visibility_tour.");
        return 1;
    }

    reportRuns(srv.response);

    if (srv.response.coverage_incomplete) {
        // Not a failure: a tour needs only one reachable configuration per
        // target, so a coverage deficit still leaves a usable tour.
        ROS_WARN("The coverage query did not reach the requested k for every "
                 "target. The tour below is still valid.");
    }

    if (!srv.response.success) {
        ROS_ERROR("Tour planning failed.");
        return 1;
    }

    ROS_INFO("Tour found: %lu waypoints, %lu stops.",
             srv.response.trajectory.points.size(),
             srv.response.stop_indices.size());
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

    return 0;
}
