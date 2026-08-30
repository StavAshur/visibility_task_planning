#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <angles/angles.h>
#include <stdexcept>
#include <utility>

// Include your headers
#include "visual_based_planning/planners/PlannerFactory.h"
#include "visual_based_planning/PlanVisibilityPath.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "../include/visual_based_planning/common/Types.h"

class VisualPlanningNode {
private:
    ros::NodeHandle nh_;  // Global Namespace ("/")
    ros::NodeHandle pnh_; // Private Namespace ("~")
    ros::ServiceServer service_;
    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
    /// The shared world. Outlives individual planners so that swapping algorithms does
    /// not rebuild the visibility structures.
    std::shared_ptr<visual_planner::PlanningContext> ctx_;
    std::unique_ptr<visual_planner::VisibilityPlannerBase> planner_;

    // State tracking to detect changes
    bool params_changed_;
    /// Set when a parameter changed that is baked into the planner object itself
    /// (its type, or a value passed at construction), so it must be rebuilt.
    bool planner_dirty_;
    bool snap_fov_on_insert_;
    bool snap_fov_on_scan_;
    bool use_visibility_integrity_;
    bool use_visibility_roadmap_;
    bool current_shortcutting_;
    std::string current_mode_;
    /// The mode the live planner_ was actually constructed for. Distinct from
    /// current_mode_ because a request may override the mode for one call.
    std::string built_mode_;

    // Planner-owned config, retained so a rebuilt planner can be configured identically.
    visual_planner::RRTParams rrt_params_;
    visual_planner::PRMParams prm_params_;
    int time_cap_;

public:
    VisualPlanningNode(planning_scene_monitor::PlanningSceneMonitorPtr psm)
        : psm_(psm), pnh_("~"), params_changed_(false), planner_dirty_(false)
    {
        ROS_WARN("Initializing Planning Service");
        planning_scene_monitor::LockedPlanningSceneRO ls(psm_);
        planning_scene::PlanningScenePtr scene_ptr = ls->diff();

        ctx_ = std::make_shared<visual_planner::PlanningContext>(scene_ptr);

        // Initialize current state trackers with defaults (or what was just loaded)
        snap_fov_on_insert_ = false;
        snap_fov_on_scan_ = false;
        use_visibility_integrity_ = false;
        use_visibility_roadmap_ = false;
        current_shortcutting_ = true;
        time_cap_ = 60;

        // Load static config (Bounds, Resolution, etc.) into the context and into the
        // retained planner config, then build the planner named by planner/mode.
        loadStaticConfig();
        if (!rebuildPlanner(scene_ptr, current_mode_)) {
            ROS_FATAL("planner/mode '%s' names no known planner. Refusing to start.",
                      current_mode_.c_str());
            throw std::runtime_error("unknown planner mode: " + current_mode_);
        }


        service_ = nh_.advertiseService("plan_visibility_path", &VisualPlanningNode::planCallback, this);
        ROS_WARN("Visual Planning Service Ready");
    }

    /**
     * @brief Refuses to start when a config still sets a parameter that was renamed.
     *
     * A parameter nobody reads is not an error ROS reports: the run would take the new
     * parameter's default and quietly do the opposite of what the config asked for.
     * Since use_visual_ik selected the VisRRT variant the paper benchmarks, a run that
     * silently ignored it would produce numbers for the wrong algorithm.
     */
    void rejectRetiredParams() {
        static const std::pair<const char*, const char*> retired[] = {
            {"planner/use_visual_ik", "planner/snap_fov_on_insert"},
        };

        for (const auto& entry : retired) {
            const std::string private_name(entry.first);
            const std::string global_name = "/" + private_name;

            if (pnh_.hasParam(private_name) || nh_.hasParam(global_name)) {
                ROS_FATAL("Parameter '%s' was renamed to '%s' and is no longer read. "
                          "Update the config rather than letting this run take a default.",
                          entry.first, entry.second);
                throw std::runtime_error("retired parameter still set: " + private_name);
            }
        }
    }

    void loadStaticConfig() {
        rejectRetiredParams();

        pnh_.param<std::string>("planner/mode", current_mode_, "VisRRT");

        double resolution;
        pnh_.param<double>("planner/resolution", resolution, 0.05);
        ctx_->setResolution(resolution);

        visual_planner::BoundingBox bounds;
        pnh_.param("planner/workspace_bounds/x_min", bounds.x_min, -2.0);
        pnh_.param("planner/workspace_bounds/x_max", bounds.x_max, 2.0);
        pnh_.param("planner/workspace_bounds/y_min", bounds.y_min, -2.0);
        pnh_.param("planner/workspace_bounds/y_max", bounds.y_max, 2.0);
        pnh_.param("planner/workspace_bounds/z_min", bounds.z_min, -0.5);
        pnh_.param("planner/workspace_bounds/z_max", bounds.z_max, 3.5);
        ctx_->setWorkspaceBounds(bounds);

        double visibility_threshold;
        pnh_.param<double>("planner/visibility_threshold", visibility_threshold, 0.75);
        ctx_->setVisibilityThreshold(visibility_threshold);

        // Retained; applied at planner construction.
        pnh_.param("planner/snap_fov_on_insert", snap_fov_on_insert_, true);
        pnh_.param("planner/snap_fov_on_scan", snap_fov_on_scan_, false);

        std::string group_name;
        // pnh_.param<std::string>("planner/group_name", group_name, "manipulator");
        pnh_.param<std::string>("planner/group_name", group_name, "whole_robot");
        ctx_->setGroupName(group_name);

        std::string ee_link;
        pnh_.param<std::string>("planner/ee_link_name", ee_link, "tool0");
        ctx_->setEELinkName(ee_link);


        visual_planner::RRTParams rrt; 
        pnh_.param("planner/rrt/goal_bias", rrt.goal_bias, 0.1);
        pnh_.param("planner/rrt/max_extension", rrt.max_extension, 0.5);
        pnh_.param("planner/rrt/max_iterations", rrt.max_iterations, 100000);
        rrt_params_ = rrt;

        visual_planner::PRMParams prm;
        pnh_.param("planner/prm/num_neighbors", prm.num_neighbors, 10);
        pnh_.param("planner/prm/num_samples", prm.num_samples, 100);
        int edge_method_int = 1; 
        pnh_.param("planner/prm/edge_validation_method", edge_method_int, 1);
        prm.edge_validation_method = static_cast<visual_planner::EdgeCheckMode>(edge_method_int);
        pnh_.param("planner/prm/max_size", prm.max_size, 10000);
        pnh_.param("planner/prm/max_goals", prm.max_goals, 10);

        prm_params_ = prm;

        visual_planner::VisibilityToolParams vt_params; // Initialized with defaults from Types.h
        pnh_.param("planner/visibility_tool_params/beam_angle", vt_params.beam_angle, vt_params.beam_angle);
        pnh_.param("planner/visibility_tool_params/beam_length", vt_params.beam_length, vt_params.beam_length);
        
        // Pass to planner
        ctx_->setVisibilityToolParams(vt_params);

        // Load Time Cap
        int time_cap;
        pnh_.param("planner/time_cap", time_cap, 60); // Default to 120 if missing
        time_cap_ = time_cap;

        bool use_visibility_integrity;
        pnh_.param("planner/use_visibility_integrity", use_visibility_integrity_, true);
        ctx_->setUseVisibilityStructure(use_visibility_integrity_);

        bool use_visibility_roadmap;
        pnh_.param("planner/use_visibility_roadmap", use_visibility_roadmap_, false);
        ctx_->setUseVisibilityRoadmap(use_visibility_roadmap_);

        visual_planner::VisibilityIntegrityParams vi_params; // Initialized with defaults from Types.h
        pnh_.param("planner/visibility_integrity_params/num_samples", vi_params.num_samples, vi_params.num_samples);
        pnh_.param("planner/visibility_integrity_params/vi_threshold", vi_params.vi_threshold, vi_params.vi_threshold);
        pnh_.param("planner/visibility_integrity_params/k_neighbors", vi_params.k_neighbors, vi_params.k_neighbors);
        
        // Pass to planner
        ctx_->setVisibilityIntegrityParams(vi_params);
    }

    /**
     * @brief Constructs the planner for `mode` on the shared context.
     *
     * Cheap: the context, and with it the visibility structures, is reused. Only the
     * per-run state is discarded, so switching algorithms or reacting to a changed
     * construction-time parameter costs one allocation rather than a VI-tree rebuild.
     *
     * @return false if `mode` names no known planner. The caller is expected to fail
     *         rather than continue: silently planning with the wrong algorithm would
     *         make a typo in planner/mode look like a working run.
     */
    bool rebuildPlanner(const planning_scene::PlanningScenePtr& scene, const std::string& mode) {
        visual_planner::PlannerOptions opts;
        opts.snap_fov_on_insert = snap_fov_on_insert_;
        opts.snap_fov_on_scan = snap_fov_on_scan_;
        opts.use_visibility_integrity = use_visibility_integrity_;
        opts.shortcutting = current_shortcutting_;
        opts.time_cap = time_cap_;
        opts.rrt = rrt_params_;
        opts.prm = prm_params_;

        auto fresh = visual_planner::createPlanner(mode, ctx_, opts);
        if (!fresh) return false;

        planner_ = std::move(fresh);
        // A new planner's nearest-neighbour index has no scene, and would silently
        // return no neighbours at all until given one.
        planner_->attachScene(scene);
        built_mode_ = mode;
        planner_dirty_ = false;
        ROS_WARN("Planner built for mode: %s", mode.c_str());
        return true;
    }

    // Helper to reload parameters and detect changes
    void updatePlannerParams() {
        bool changed_this_cycle = false;

        // 1. FOV snapping (Global)
        bool new_snap_insert = true;
        if (!nh_.getParam("/planner/snap_fov_on_insert", new_snap_insert)) {
            new_snap_insert = true;
        }

        if (new_snap_insert != snap_fov_on_insert_) {
            // Passed at construction, so the planner has to be rebuilt to pick it up.
            planner_dirty_ = true;
            snap_fov_on_insert_ = new_snap_insert;
            changed_this_cycle = true;
            ROS_INFO("Param Changed: snap_fov_on_insert -> %s", new_snap_insert ? "TRUE" : "FALSE");
        }

        bool new_snap_scan = false;
        if (!nh_.getParam("/planner/snap_fov_on_scan", new_snap_scan)) {
            new_snap_scan = false;
        }

        if (new_snap_scan != snap_fov_on_scan_) {
            planner_->setSnapFovOnScan(new_snap_scan);
            snap_fov_on_scan_ = new_snap_scan;
            changed_this_cycle = true;
            ROS_INFO("Param Changed: snap_fov_on_scan -> %s", new_snap_scan ? "TRUE" : "FALSE");
        }

        // 2. Visibility Integrity (Global)
        bool new_vi = false;
        nh_.getParam("/planner/use_visibility_integrity", new_vi);
        // if (nh_.getParam("/planner/use_visibility_integrity", new_vi)) {
        //     // Found specific param
        // } 
        // else if (nh_.getParam("/planner/visibility_integrity/enabled", new_vi)) {
        //     // Found nested param
        // }

        if (new_vi != use_visibility_integrity_) {
            planner_->setUseVisibilityIntegrity(new_vi);
            use_visibility_integrity_ = new_vi;
            changed_this_cycle = true;
            ROS_INFO("Param Changed: use_visibility_integrity -> %s", new_vi ? "TRUE" : "FALSE");
        }

        // 3. Visibility Roadmap (Global)
        bool new_vr = false;
        nh_.getParam("/planner/use_visibility_roadmap", new_vr);

        if (new_vr != use_visibility_roadmap_) {
            planner_->setUseVisibilityRoadmap(new_vr);
            use_visibility_roadmap_ = new_vr;
            changed_this_cycle = true;
            ROS_INFO("Param Changed: use_visibility_roadmap -> %s", new_vr ? "TRUE" : "FALSE");
        }


        // 4. Shortcutting (Global)
        bool new_shortcutting = true;
        nh_.getParam("/planner/enable_shortcutting", new_shortcutting);

        if (new_shortcutting != current_shortcutting_) {
            planner_->setShortcutting(new_shortcutting);
            current_shortcutting_ = new_shortcutting;
            changed_this_cycle = true;
            ROS_INFO("Param Changed: enable_shortcutting -> %s", new_shortcutting ? "TRUE" : "FALSE");
        }

        // 5. Mode (Global)
        std::string new_mode;
        nh_.getParam("planner/mode", new_mode);
        if (new_mode != current_mode_) {
            current_mode_ = new_mode;
            changed_this_cycle = true;
            ROS_INFO("Param Changed: planner/mode -> %s", current_mode_.c_str());
        }
    

        // Update the member flag if any change occurred
        if (changed_this_cycle) {
            params_changed_ = true;
        }
    }

    bool planCallback(visual_based_planning::PlanVisibilityPath::Request &req,
                      visual_based_planning::PlanVisibilityPath::Response &res) {
        
        // 1. Check for param updates
        updatePlannerParams();

        // 2. Determine Mode. The request overrides the parameter server for this call
        // only, so the mode is resolved before the planner is chosen.
        std::string mode = current_mode_;
        if (!req.planner_type.empty()) {
             mode = req.planner_type;
        }

        planning_scene_monitor::LockedPlanningSceneRO ls(psm_);
        planning_scene::PlanningScenePtr fresh_scene;

        // 3. Rebuild the shared context if it is stale. This is the expensive path: it
        // re-extracts obstacles and rebuilds the visibility structure.
        if (!ctx_->isInitialized() || params_changed_) {

            ROS_WARN("Context Re-Initialization Triggered (Init: %d, ParamsChanged: %d)",
                     ctx_->isInitialized(), params_changed_);

            planner_->reset();

            fresh_scene = ls->diff();
            ctx_->setPlanningScene(fresh_scene);
            planner_->attachScene(fresh_scene);

            params_changed_ = false;
        }

        // 4. Swap the planner if the requested mode differs from the one we built, or
        // if a construction-time parameter changed. The context is untouched.
        if (planner_dirty_ || mode != built_mode_) {
            if (!fresh_scene) fresh_scene = ls->diff();
            if (!rebuildPlanner(fresh_scene, mode)) {
                ROS_ERROR("Requested planner '%s' is unknown; failing this request.",
                          mode.c_str());
                res.success = false;
                return true;
            }
        }

        // 5. Pass Targets. Done after any rebuild -- the target and start configuration
        // are per-run state and would not survive one.
        planner_->computeTargetMES(req.task.target_points);

        if (!req.task.start_joints.empty()) {
            planner_->setStartJoints(req.task.start_joints);
        } else {
            std::vector<double> current_joints;
            std::string group = planner_->getGroupName();
            ls->getCurrentState().copyJointGroupPositions(group, current_joints);
            planner_->setStartJoints(current_joints);
        }

        ROS_WARN("Executing Planner with Mode: %s", mode.c_str());

        bool success = planner_->plan();

        res.success = success;
        
        if (success) {
            const auto& raw_path = planner_->getResultPath();
            std::string group = planner_->getGroupName();
            const moveit::core::JointModelGroup* jmg = ls->getRobotModel()->getJointModelGroup(group);
            res.trajectory.joint_names = jmg->getActiveJointModelNames();
            for (const auto& conf : raw_path) {
                trajectory_msgs::JointTrajectoryPoint point;
                point.positions = conf;
                res.trajectory.points.push_back(point);
            }
        } else {
            ROS_WARN("Planner failed to find a solution.");
        }
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "visual_planning_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2); 
    spinner.start();
    
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(
        new robot_model_loader::RobotModelLoader("robot_description"));

    planning_scene_monitor::PlanningSceneMonitorPtr psm(
        new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    
    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->startStateMonitor();

    VisualPlanningNode node(psm);

    ros::waitForShutdown();
    
    return 0;
}
