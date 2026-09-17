#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <angles/angles.h>
#include <stdexcept>
#include <utility>

// Include your headers
#include "visual_based_planning/planners/PlannerFactory.h"
#include "visual_based_planning/planners/LocalVisTSP.h"
#include "visual_based_planning/PlanVisibilityPath.h"
#include "visual_based_planning/PlanVisibilityTour.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "../include/visual_based_planning/common/Types.h"

class VisualPlanningNode {
private:
    ros::NodeHandle nh_;  // Global Namespace ("/")
    ros::NodeHandle pnh_; // Private Namespace ("~")
    ros::ServiceServer service_;
    /// The VisTSP tour service (global_plan step 4). Separate from service_
    /// because its result -- per-algorithm costs, stops and leg boundaries --
    /// does not fit the single-target response, and the single-target path
    /// serves the TVMP paper and should not change shape for this.
    ros::ServiceServer tour_service_;
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
    bool fix_base_on_ik_ = false;
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

    // --- VisTSP tour config (planner/vistsp/*), re-read on every tour call ---
    // These are parameters rather than request fields on purpose: ROS messages
    // have no optional fields, so an unset bool would arrive as false and
    // silently switch an algorithm off. Parameters also match how every other
    // knob in this node works, and experiment_runner.cpp already sets
    // parameters before calling, so sweeps need no new machinery.
    bool vistsp_use_nearest_;
    bool vistsp_use_farthest_;
    bool vistsp_use_cheapest_;
    bool vistsp_use_two_opt_;
    int  vistsp_coverage_;
    /// Radius in metres the base is confined to, measured from the start
    /// configuration. <= 0 means UNCONSTRAINED, which runs the same pipeline
    /// over the whole environment -- the intended VisCfgTSP baseline, not a
    /// misconfiguration.
    double vistsp_base_locality_radius_;

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
        vistsp_use_nearest_ = true;
        vistsp_use_farthest_ = true;
        vistsp_use_cheapest_ = true;
        vistsp_use_two_opt_ = true;
        vistsp_coverage_ = 1;
        vistsp_base_locality_radius_ = 0.0;

        // Load static config (Bounds, Resolution, etc.) into the context and into the
        // retained planner config, then build the planner named by planner/mode.
        loadStaticConfig();
        if (!rebuildPlanner(scene_ptr, current_mode_)) {
            ROS_FATAL("planner/mode '%s' names no known planner. Refusing to start.",
                      current_mode_.c_str());
            throw std::runtime_error("unknown planner mode: " + current_mode_);
        }


        service_ = nh_.advertiseService("plan_visibility_path", &VisualPlanningNode::planCallback, this);
        tour_service_ = nh_.advertiseService("plan_visibility_tour", &VisualPlanningNode::tourCallback, this);
        ROS_WARN("Visual Planning Service Ready (plan_visibility_path, plan_visibility_tour)");
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
        // Construction-time, like the snap flags: it is passed to the planner when
        // one is built, so changing it takes effect on the next planner rebuild.
        pnh_.param("planner/fix_base_on_ik", fix_base_on_ik_, false);

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

        loadTourConfig();
    }

    /**
     * @brief Loads the planner/vistsp parameters, the VisTSP tour knobs.
     *
     * Called from loadStaticConfig() and again at the start of every tour
     * request, so a sweep can change the algorithm set between calls without
     * restarting the node. None of these force a planner rebuild: they are read
     * by LocalVisTSP, or applied to the live planner, not baked in at
     * construction.
     */
    void loadTourConfig() {
        pnh_.param("planner/vistsp/use_nearest_insertion",  vistsp_use_nearest_,  true);
        pnh_.param("planner/vistsp/use_farthest_insertion", vistsp_use_farthest_, true);
        pnh_.param("planner/vistsp/use_cheapest_insertion", vistsp_use_cheapest_, true);
        pnh_.param("planner/vistsp/use_two_opt",            vistsp_use_two_opt_,  true);
        pnh_.param("planner/vistsp/coverage",               vistsp_coverage_,     1);
        pnh_.param("planner/vistsp/base_locality_radius",
                   vistsp_base_locality_radius_, 0.0);
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
        opts.fix_base_on_ik = fix_base_on_ik_;
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

    /**
     * @brief Setup shared by both service callbacks: refresh parameters, resolve
     *        the mode, re-initialise the context if stale, swap the planner if
     *        needed.
     *
     * Extracted when the tour service was added (2026-09-09). Behaviour is
     * unchanged; the point is that two service handlers must not each carry
     * their own copy of this sequence, because the copies would drift and one
     * service would quietly plan with stale parameters.
     *
     * @param requested_mode Overrides planner/mode for this call only; may be empty.
     * @param ls             The caller's scene lock, held for the whole request.
     * @param mode_out       Receives the mode actually used.
     * @return false if the requested mode names no known planner.
     */
    bool preparePlanner(const std::string& requested_mode,
                        planning_scene_monitor::LockedPlanningSceneRO& ls,
                        std::string& mode_out) {
        // 1. Check for param updates
        updatePlannerParams();

        // 2. Determine Mode. The request overrides the parameter server for this call
        // only, so the mode is resolved before the planner is chosen.
        mode_out = current_mode_;
        if (!requested_mode.empty()) {
            mode_out = requested_mode;
        }

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
        if (planner_dirty_ || mode_out != built_mode_) {
            if (!fresh_scene) fresh_scene = ls->diff();
            if (!rebuildPlanner(fresh_scene, mode_out)) {
                ROS_ERROR("Requested planner '%s' is unknown; failing this request.",
                          mode_out.c_str());
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Applies the request's start joints, or the robot's current state.
     *
     * Must run AFTER any planner rebuild: the start configuration is per-run
     * state and would not survive one.
     */
    void applyStartJoints(const std::vector<double>& requested,
                          planning_scene_monitor::LockedPlanningSceneRO& ls) {
        if (!requested.empty()) {
            planner_->setStartJoints(requested);
        } else {
            std::vector<double> current_joints;
            std::string group = planner_->getGroupName();
            ls->getCurrentState().copyJointGroupPositions(group, current_joints);
            planner_->setStartJoints(current_joints);
        }
    }

    /**
     * @brief Pulls the current planning scene from move_group before it is read.
     *
     * WHY. The monitor is a SUBSCRIBER: it only ever learns about world
     * geometry published after it connected. Anything applied to move_group
     * before this node started -- a scene loaded ahead of the planner, or a
     * planner restarted while the scene stayed up -- was invisible here, and
     * the node then planned in an empty world without ever saying so. That made
     * the bring-up order load-bearing, and getting it wrong produced plausible
     * paths through walls rather than an error.
     *
     * One /get_planning_scene round trip, milliseconds against a query capped
     * at planner/time_cap seconds, so it is not worth making conditional.
     *
     * MUST be called BEFORE LockedPlanningSceneRO: the request takes the
     * scene's write lock to apply what comes back, so calling it while holding
     * the read lock deadlocks.
     */
    void refreshPlanningScene() {
        if (!psm_->requestPlanningSceneState()) {
            // Not fatal: the monitor may still be up to date from the topic.
            // Loud, because the alternative explanation is an empty world.
            ROS_WARN("Could not pull the planning scene from move_group "
                     "(/get_planning_scene). Continuing with whatever the scene "
                     "monitor has received; if the world looks empty, that is why.");
        }
    }

    bool planCallback(visual_based_planning::PlanVisibilityPath::Request &req,
                      visual_based_planning::PlanVisibilityPath::Response &res) {

        refreshPlanningScene();
        planning_scene_monitor::LockedPlanningSceneRO ls(psm_);
        std::string mode;
        if (!preparePlanner(req.planner_type, ls, mode)) {
            res.success = false;
            return true;
        }

        // 5. Pass Targets. Done after any rebuild -- the target and start configuration
        // are per-run state and would not survive one.
        planner_->computeTargetMES(req.task.target_points);
        applyStartJoints(req.task.start_joints, ls);

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

    /**
     * @brief Plans a visibility tour over several targets (global_plan step 4).
     *
     * Runs planners/LocalVisTSP.h: a multi-target coverage query, then the
     * enabled E-GTSP insertion rules, each optionally improved by 2-opt, then
     * the cheapest tour expanded into a trajectory and smoothed per leg.
     *
     * WHAT vs HOW. The request says what to solve -- the targets, the start,
     * the planner. How to solve it comes from the planner/vistsp parameters,
     * reloaded here
     * on every call so a sweep can change the algorithm set between requests
     * without restarting the node.
     *
     * LOCAL OR GLOBAL. planner/vistsp/base_locality_radius <= 0 leaves the base
     * unconstrained and tours the whole environment. That is the intended
     * baseline (the VisCfgTSP benchmark), so it is logged as a deliberate mode
     * rather than treated as a missing setting.
     */
    bool tourCallback(visual_based_planning::PlanVisibilityTour::Request &req,
                      visual_based_planning::PlanVisibilityTour::Response &res) {
        res.success = false;
        res.best = -1;
        res.coverage_incomplete = false;

        if (req.targets.empty()) {
            ROS_ERROR("Tour request carries no targets; refusing. VisTSP needs the "
                      "targets named individually, unlike the single-target service "
                      "which collapses a point cloud into one sphere.");
            return true;
        }

        loadTourConfig();

        refreshPlanningScene();
        planning_scene_monitor::LockedPlanningSceneRO ls(psm_);
        std::string mode;
        if (!preparePlanner(req.planner_type, ls, mode)) {
            return true;
        }

        // --- per-run state, applied after any planner rebuild ---------------
        std::vector<visual_planner::Ball> targets;
        std::vector<geometry_msgs::Point> centers;
        targets.reserve(req.targets.size());
        centers.reserve(req.targets.size());
        for (const auto& t : req.targets) {
            visual_planner::Ball ball;
            ball.center = Eigen::Vector3d(t.center.x, t.center.y, t.center.z);
            ball.radius = t.radius;
            targets.push_back(ball);
            centers.push_back(t.center);
        }
        planner_->setTargets(targets);

        // The coverage query works off setTargets(), but several planner
        // helpers still read target_mes_. Set it to a sphere enclosing every
        // target centre so it is not left at whatever a previous single-target
        // request happened to leave behind.
        planner_->computeTargetMES(centers);

        planner_->setCoverage(vistsp_coverage_);
        if (vistsp_base_locality_radius_ > 0.0) {
            planner_->setBaseLocality(vistsp_base_locality_radius_);
            ROS_WARN("Tour: LOCAL run, base confined to %.2f m of the start.",
                     vistsp_base_locality_radius_);
        } else {
            planner_->clearBaseLocality();
            ROS_WARN("Tour: GLOBAL run, base unconstrained (the VisCfgTSP baseline).");
        }

        applyStartJoints(req.start_joints, ls);

        // --- solve -----------------------------------------------------------
        visual_planner::LocalVisTSP solver(*planner_);
        solver.setUseNearestInsertion(vistsp_use_nearest_);
        solver.setUseFarthestInsertion(vistsp_use_farthest_);
        solver.setUseCheapestInsertion(vistsp_use_cheapest_);
        solver.setUseTwoOpt(vistsp_use_two_opt_);

        ROS_WARN("Executing VisTSP tour with Mode: %s over %lu targets "
                 "(nearest=%d farthest=%d cheapest=%d two_opt=%d, coverage=%d)",
                 mode.c_str(), req.targets.size(),
                 vistsp_use_nearest_, vistsp_use_farthest_,
                 vistsp_use_cheapest_, vistsp_use_two_opt_, vistsp_coverage_);

        const bool success = solver.solve();
        const visual_planner::LocalVisTSPResult& result = solver.getResult();

        res.coverage_incomplete = result.coverage_incomplete;
        res.coverage_seconds = result.coverage_seconds;
        res.instance_seconds = result.instance_seconds;
        res.gtsp_seconds     = result.gtsp_seconds;
        res.expand_seconds   = result.expand_seconds;
        res.total_seconds    = result.total_seconds;

        // The per-rule costs are reported whether or not a tour came out: a
        // failure after some rules succeeded is still informative.
        for (const auto& run : result.runs) {
            res.algorithm_names.push_back(run.name);
            res.construction_costs.push_back(run.construction_cost);
            res.improved_costs.push_back(run.improved_cost);
        }
        res.best = result.best;

        if (!success) {
            ROS_WARN("VisTSP tour failed.");
            return true;
        }

        // --- fill the trajectory, the same conversion planCallback uses ------
        const moveit::core::JointModelGroup* jmg =
            ls->getRobotModel()->getJointModelGroup(planner_->getGroupName());
        res.trajectory.joint_names = jmg->getActiveJointModelNames();
        for (const auto& conf : result.path) {
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions = conf;
            res.trajectory.points.push_back(point);
        }

        // Stops and the target each one observes, so a client can draw which
        // configuration is watching what.
        for (size_t i = 0; i < result.stop_indices.size(); ++i) {
            res.stop_indices.push_back(result.stop_indices[i]);
            const int stop_node = (i < result.runs[result.best].tour.stops.size())
                                ? result.runs[result.best].tour.stops[i] : -1;
            res.stop_target.push_back(
                (stop_node >= 0 &&
                 stop_node < static_cast<int>(result.instance.node_target.size()))
                    ? result.instance.node_target[stop_node] : -1);
        }

        res.success = true;
        ROS_WARN("VisTSP tour: %s won with cost %.4f over %lu waypoints and %lu stops.",
                 result.runs[result.best].name,
                 result.runs[result.best].improved_cost,
                 result.path.size(), result.stop_indices.size());
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

    // Adopt whatever move_group already holds, rather than only what is
    // published from now on. Without this the node starts blind to a scene that
    // was loaded before it, which is why the bring-up order used to matter; see
    // VisualPlanningNode::refreshPlanningScene(). Failure is expected and
    // harmless when move_group is not up yet -- the service callbacks ask again.
    if (!psm->requestPlanningSceneState()) {
        ROS_WARN("No planning scene from move_group at startup. It will be "
                 "requested again on the first service call.");
    }

    VisualPlanningNode node(psm);

    ros::waitForShutdown();
    
    return 0;
}
