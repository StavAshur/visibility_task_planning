#pragma once

/**
 * @file RunSetup.h
 * @brief Everything a VisTSP run needs set up before it can be planned:
 *        the sampling regions, the scene obstacles, the targets drawn from them,
 *        the start configuration, and the markers that draw the targets.
 *
 * WHY THIS IS SHARED. Two programs set a run up: src/vistsp_experiment_runner.cpp
 * (many trials, many methods, statistics) and src/visual_planning_tour_client.cpp
 * (one run, executed on the robot and watched). They must set it up THE SAME WAY,
 * or the single run being watched is not a member of the distribution being
 * benchmarked -- it would look like the experiment while quietly being a different
 * one. Concretely: with the same regions, the same seed and the same target count,
 * the tour client draws EXACTLY the targets of the experiment's trial 0, because
 * the draw below is the only implementation of "how a target is drawn" and it
 * consumes the random engine in a fixed order (region, x, y, z, repeat).
 *
 * Nothing here knows about tours or planners. It is the boundary between "what
 * problem are we solving" and "how do we solve it".
 */

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <random>
#include <string>
#include <vector>

namespace visual_planner {
namespace run_setup {

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/// Axis-aligned box. Used for both a sampling region and a scene obstacle: the
/// two are tested against each other, so they share a shape.
struct AABB {
    double min_x, max_x;
    double min_y, max_y;
    double min_z, max_z;
};

/// One target ball, plus which region produced it. The region id is carried all
/// the way into the experiment CSV, so a result can be traced back to where its
/// target came from.
struct TargetSphere {
    double cx = 0.0, cy = 0.0, cz = 0.0;
    double radius = 0.0;
    int region_id = -1;
};

/// Closest-point test: a sphere meets a box when the box point nearest its
/// centre is within the radius.
inline bool sphereIntersectsAABB(double cx, double cy, double cz, double r,
                                 const AABB& b) {
    const double x = std::max(b.min_x, std::min(cx, b.max_x));
    const double y = std::max(b.min_y, std::min(cy, b.max_y));
    const double z = std::max(b.min_z, std::min(cz, b.max_z));
    const double dx = cx - x, dy = cy - y, dz = cz - z;
    return (dx * dx + dy * dy + dz * dz) <= r * r;
}

// ---------------------------------------------------------------------------
// The world the targets are drawn from
// ---------------------------------------------------------------------------

/**
 * @brief Loads sampling regions from a YAML file with a `sampling_regions:` list
 *        of {min: [x,y,z], max: [x,y,z]}.
 * @return false on a read error or an empty list, which is fatal to a run: there
 *         would be nowhere to put a target.
 */
inline bool loadRegions(const std::string& file, std::vector<AABB>& out) {
    try {
        YAML::Node config = YAML::LoadFile(file);
        for (const auto& node : config["sampling_regions"]) {
            AABB r;
            r.min_x = node["min"][0].as<double>();
            r.min_y = node["min"][1].as<double>();
            r.min_z = node["min"][2].as<double>();
            r.max_x = node["max"][0].as<double>();
            r.max_y = node["max"][1].as<double>();
            r.max_z = node["max"][2].as<double>();
            out.push_back(r);
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to load sampling regions from '%s': %s", file.c_str(), e.what());
        return false;
    }
    if (out.empty()) {
        ROS_ERROR("No sampling regions in '%s'.", file.c_str());
        return false;
    }
    ROS_INFO("Loaded %lu sampling regions from %s.", out.size(), file.c_str());
    return true;
}

/// Scene obstacles as AABBs, so a sampled target can be rejected if it is buried
/// in one. BOX primitives only, and their rotation is ignored -- the same
/// simplification PlanningContext::initialize_visibility() makes.
inline void collectSceneObstacles(std::vector<AABB>& out) {
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
            out.push_back(o);
        }
    }
    ROS_INFO("Extracted %lu box obstacles from the planning scene.", out.size());
}

/**
 * @brief Blocks until the planning scene holds at least `min_objects` objects.
 *
 * An empty world has no occlusion, so every target is trivially visible and a run
 * still produces a full, plausible, meaningless result. Waiting - and refusing
 * when the wait expires - is what keeps a forgotten scene from looking like data.
 *
 * @return false when the scene never arrived.
 */
inline bool waitForSceneObjects(int min_objects, double timeout_s) {
    if (min_objects <= 0) {
        ROS_WARN("Not waiting for a scene. The run will use whatever world is "
                 "loaded, including an empty one.");
        return true;
    }
    moveit::planning_interface::PlanningSceneInterface psi;
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(timeout_s);
    ros::WallTime next_log = ros::WallTime::now();
    while (ros::ok()) {
        const std::vector<std::string> known = psi.getKnownObjectNames();
        if (static_cast<int>(known.size()) >= min_objects) {
            ROS_INFO("Planning scene holds %lu objects.", known.size());
            return true;
        }
        if (ros::WallTime::now() > deadline) {
            ROS_FATAL("Waited %.0f s and the planning scene still holds %lu objects "
                      "(< %d). Load the environment, e.g. 'roslaunch scene_builder "
                      "load_scene.launch'.", timeout_s, known.size(), min_objects);
            return false;
        }
        if (ros::WallTime::now() >= next_log) {
            ROS_INFO("Waiting for the planning scene (%lu/%d objects)...",
                     known.size(), min_objects);
            next_log = ros::WallTime::now() + ros::WallDuration(5.0);
        }
        ros::Duration(0.5).sleep();
    }
    return false;
}

// ---------------------------------------------------------------------------
// The draw
// ---------------------------------------------------------------------------

/**
 * @brief Draws target balls from the regions. THE definition of that operation.
 *
 * One target: pick a region uniformly - BY REGION, not by volume, so a small
 * region gets as many targets as a large one - then a centre uniformly inside it,
 * then the radius is the shortest distance from that centre to the region's own
 * bounds, capped by max_radius. A sphere that meets a scene obstacle is discarded
 * and the whole draw is retried, region included.
 *
 * The bounds rule is what keeps a target inside the band it came from; the cap
 * only ever makes it smaller. Regions are drawn WITH REPETITION across a set of
 * targets, so two of them may overlap - that is wanted, being exactly the case
 * where one configuration sees two targets.
 */
class TargetSampler {
public:
    TargetSampler(const std::vector<AABB>& regions, const std::vector<AABB>& obstacles,
                  unsigned seed, double max_radius)
        : regions_(regions), obstacles_(obstacles), rng_(seed), max_radius_(max_radius) {}

    /// @return false only after 1000 rejected draws, which means the regions are
    ///         effectively inside obstacles rather than that this draw was unlucky.
    bool sample(TargetSphere& out) {
        if (regions_.empty()) return false;
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
            t.radius = std::min(t.radius, max_radius_);
            if (t.radius <= 0.0) continue;

            bool hit = false;
            for (size_t o = 0; o < obstacles_.size() && !hit; ++o) {
                if (sphereIntersectsAABB(t.cx, t.cy, t.cz, t.radius, obstacles_[o])) hit = true;
            }
            if (hit) continue;

            out = t;
            return true;
        }
        return false;
    }

    /// True when any two of these targets intersect. Reported rather than
    /// prevented: an overlap is what exercises node splitting.
    static bool anyOverlap(const std::vector<TargetSphere>& targets) {
        for (size_t a = 0; a < targets.size(); ++a) {
            for (size_t b = a + 1; b < targets.size(); ++b) {
                const TargetSphere& u = targets[a];
                const TargetSphere& v = targets[b];
                const double dx = u.cx - v.cx, dy = u.cy - v.cy, dz = u.cz - v.cz;
                if (std::sqrt(dx * dx + dy * dy + dz * dz) < u.radius + v.radius) return true;
            }
        }
        return false;
    }

private:
    const std::vector<AABB>& regions_;
    const std::vector<AABB>& obstacles_;
    std::mt19937 rng_;
    double max_radius_;
};

// ---------------------------------------------------------------------------
// The robot's starting point
// ---------------------------------------------------------------------------

/// Loads robot_description once, for the marker frame and the start state.
/// A null return is fatal to the caller: without the model there is neither a
/// frame to draw in nor a start configuration to plan from.
inline robot_model::RobotModelPtr loadRobotModel() {
    robot_model_loader::RobotModelLoader loader("robot_description");
    robot_model::RobotModelPtr model = loader.getModel();
    if (!model) {
        ROS_FATAL("Could not load 'robot_description'. Is MoveIt running? The VisTSP "
                  "runs need mobile_ur_moveit_config demo.launch (or the robot's "
                  "bring-up) started first.");
    }
    return model;
}

/**
 * @brief Resolves which planning group a named start state belongs to.
 *
 * Order: the caller's own parameter, then the NODE's planner/group_name. The node
 * is the authority - it is the process that plans - so its value beats a default
 * repeated in a client. Neither set is a refusal rather than a guess: the wrong
 * group would resolve the state name against the wrong joints.
 */
inline bool resolvePlanningGroup(ros::NodeHandle& nh, std::string& group) {
    if (!group.empty()) return true;
    if (nh.getParam("/visual_planning_node/planner/group_name", group) && !group.empty()) {
        ROS_INFO("Planning group taken from the node: '%s'.", group.c_str());
        return true;
    }
    ROS_FATAL("No planning group: /visual_planning_node/planner/group_name is unset "
              "and no ~planning_group was given.");
    return false;
}

/**
 * @brief Resolves an SRDF group_state into the joint vector a run starts from.
 *
 * A NAMED state, never a vector in YAML: "whole_robot" is built from subgroups, so
 * a hand-written list would silently depend on MoveIt's variable order. Going
 * through RobotState uses the same order the planner's copyJointGroupPositions()
 * does, by construction.
 *
 * An unknown name is FATAL and lists what exists. Falling back to the robot's
 * current state would still produce a run, from a start nobody chose.
 *
 * @param joints Receives the group's positions; @param names the matching variable
 *        names, which is what lets the caller publish the state to park the robot.
 */
inline bool resolveNamedStartState(const robot_model::RobotModelPtr& model,
                                   const std::string& group,
                                   const std::string& state_name,
                                   std::vector<double>& joints,
                                   std::vector<std::string>& names) {
    const moveit::core::JointModelGroup* jmg = model->getJointModelGroup(group);
    if (!jmg) {
        ROS_FATAL("Group '%s' does not exist in the robot model.", group.c_str());
        return false;
    }
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    if (!state.setToDefaultValues(jmg, state_name)) {
        std::string known;
        const std::vector<std::string>& all = jmg->getDefaultStateNames();
        for (size_t i = 0; i < all.size(); ++i) known += (i ? ", " : "") + all[i];
        ROS_FATAL("Group '%s' has no state named '%s'. Known states: [%s].",
                  group.c_str(), state_name.c_str(), known.c_str());
        return false;
    }
    state.update();
    state.copyJointGroupPositions(jmg, joints);
    names = jmg->getVariableNames();

    std::string text;
    for (size_t i = 0; i < joints.size(); ++i) {
        char buf[64];
        snprintf(buf, sizeof(buf), "%s%.3f", i ? ", " : "", joints[i]);
        text += buf;
    }
    ROS_INFO("Start state '%s' of group '%s' resolved to [%s].",
             state_name.c_str(), group.c_str(), text.c_str());
    return true;
}

/**
 * @brief Moves the SIMULATED robot to a configuration, by publishing it to the
 *        fake controller topic demo.launch's joint_state_publisher merges.
 *
 * Simulation only, and never load-bearing: the run sends its start configuration
 * to the planner explicitly, so a robot that did not move plans from the same
 * place regardless. It matters for what is SEEN - and for the planning scene's
 * current state, which the planner snapshots and which seeds collision checking
 * for joints outside the planning group.
 */
inline void parkSimulatedRobot(ros::NodeHandle& nh,
                               const std::vector<std::string>& names,
                               const std::vector<double>& joints,
                               const std::string& state_name) {
    if (joints.empty() || names.size() != joints.size()) return;

    ros::Publisher pub =
        nh.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1);

    sensor_msgs::JointState js;
    js.name = names;
    js.position = joints;

    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(5.0);
    while (ros::ok() && pub.getNumSubscribers() == 0 && ros::WallTime::now() < deadline) {
        ros::Duration(0.1).sleep();
    }
    if (pub.getNumSubscribers() == 0) {
        ROS_WARN("Nothing subscribes to /move_group/fake_controller_joint_states, so the "
                 "robot was NOT parked at '%s'. Harmless on hardware and without "
                 "demo.launch: the request carries the start configuration anyway.",
                 state_name.c_str());
        return;
    }
    // Repeated because the joint_state_publisher merges its sources on its own
    // timer; one message can land between two of its publications.
    for (int i = 0; i < 10 && ros::ok(); ++i) {
        js.header.stamp = ros::Time::now();
        pub.publish(js);
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("Parked the simulated robot at '%s'.", state_name.c_str());
}

// ---------------------------------------------------------------------------
// Drawing and reporting the targets
// ---------------------------------------------------------------------------

/// The targets as three numbers each, so a log can be checked against what is on
/// screen and against what the planner prints from beginCoverageRun(). The index
/// is the one the marker labels, the deficit reports and the tour response's
/// stop_target all use.
inline void logTargets(const std::vector<TargetSphere>& targets,
                       const std::string& frame, const std::string& what) {
    ROS_INFO("%s: %lu targets (frame %s)%s:", what.c_str(), targets.size(),
             frame.empty() ? "<unset>" : frame.c_str(),
             TargetSampler::anyOverlap(targets) ? ", CONTAINS OVERLAPPING TARGETS" : "");
    for (size_t i = 0; i < targets.size(); ++i) {
        ROS_INFO("  target %lu: center (%.4f, %.4f, %.4f), radius %.4f  [region %d]",
                 i, targets[i].cx, targets[i].cy, targets[i].cz,
                 targets[i].radius, targets[i].region_id);
    }
}

/**
 * @brief The target spheres as RViz markers: a translucent ball at each target's
 *        true radius, plus its INDEX as a text label.
 *
 * The labels are not decoration: the tour response's stop_target refers to those
 * indices, so without them a tour cannot be read off the screen. A DELETEALL
 * leads, so a previous run's targets cannot be mistaken for this one's.
 */
inline visualization_msgs::MarkerArray targetMarkers(const std::vector<TargetSphere>& targets,
                                                     const std::string& frame) {
    visualization_msgs::MarkerArray array;

    visualization_msgs::Marker clear;
    clear.header.frame_id = frame;
    clear.header.stamp = ros::Time::now();
    clear.action = visualization_msgs::Marker::DELETEALL;
    array.markers.push_back(clear);

    for (size_t i = 0; i < targets.size(); ++i) {
        const TargetSphere& t = targets[i];

        visualization_msgs::Marker m;
        m.header.frame_id = frame;
        m.header.stamp = ros::Time::now();
        m.ns = "vistsp_targets";
        m.id = static_cast<int>(i);
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = t.cx;
        m.pose.position.y = t.cy;
        m.pose.position.z = t.cz;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 2.0 * t.radius;   // diameter
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
    return array;
}

} // namespace run_setup
} // namespace visual_planner
