#pragma once

#include "VisibilityPlannerBase.h"

namespace visual_planner {

/**
 * @brief Visibility PRM.
 *
 * Builds a roadmap, connects the start configuration into it, and scans both the
 * existing roadmap and newly sampled configurations for one that sees enough of the
 * target ball. When visibility guidance is enabled the goal samples are drawn from
 * the context's visibility structure rather than uniformly, which is what
 * distinguishes VisPRM from a plain PRM.
 *
 * Intended as a base for VisPRM variants; the primitives it is written in terms of
 * live in VisibilityPlannerBase.
 */
class VisPRMPlanner : public VisibilityPlannerBase {
public:
    explicit VisPRMPlanner(std::shared_ptr<PlanningContext> ctx)
        : VisibilityPlannerBase(std::move(ctx)) {}

    /**
     * @brief Plans from the configured start, sampling one if none was set.
     */
    bool plan() override {

        // Use member variable start_joint_values_
        if (start_joint_values_.empty()) {
            ROS_WARN("Start joint values not set! Sampling random start configuration...");
            std::vector<double> start_joint_values;
	        bool found_random_start = false;

            while (!found_random_start) {
                start_joint_values = ctx_->sampler_->sampleUniform();
                found_random_start = ctx_->validity_checker_->isValid(start_joint_values);
                // Specifically for the experiments in the double room env
                found_random_start = !((start_joint_values[0] > 0) && (start_joint_values[0] < 2.25) && (start_joint_values[1] < 2.25) && (start_joint_values[1] > -2.25));
            } 
            return planFrom(start_joint_values);
        }

        return planFrom(start_joint_values_);
    }

protected:
    /**
     * @brief The roadmap search proper, from an explicit start configuration.
     */
    bool planFrom(std::vector<double> start_joint_values) {
        
        if (start_joint_values.empty()) { ROS_ERROR("Start joint values not set!"); return false; }
	
        // 1. Add Start Configuration
        root_id_ = addState(start_joint_values);
        checked_vertices_.clear();
	    ROS_WARN("root_id is set to: %zu", root_id_);  

        
        size_t start_num_neighbors = (graph_.getNumVertices() > 100) ? prm_params_.num_neighbors : 
                                                                        prm_params_.num_neighbors * 3 ;
        
        VertexDesc res = connectToGraph(start_joint_values, start_num_neighbors, root_id_);
        
        bool start_cfg_connected = (res != -1);

        std::string success = "SUCCESS";
        std::string failure = "FAILURE";
    
        ROS_WARN("%s connecting start configuration to roadmap", start_cfg_connected ? success.c_str() : failure.c_str());

        if (root_id_ == -1) { ROS_ERROR("Start state is invalid!"); return false; }

        ctx_->robot_state_->setJointGroupPositions(ctx_->group_name_, start_joint_values);
        ctx_->robot_state_->update();
        
        std::vector<VertexDesc> goal_ids;

        ROS_INFO("Starting VisPRM...");
        ros::WallTime start_time = ros::WallTime::now();

        ROS_INFO("Scanning existing roadmap for target visibility...");
        
        // Assuming GraphManager exposes the number of vertices. 
        // If not, use appropriate iterator from your Graph.h
        size_t num_vertices = graph_.getNumVertices(); 
        
        for (size_t i = 0; i < num_vertices; ++i) {
            std::vector<double> q_node = graph_.getVertexConfig(i);

            // Actual Visibility Oracle Check
            // Check if this node actually sees the target
            double vis_score = ctx_->vis_oracle_->checkBallBeamVisibility(graph_.getVertexPose(i), target_mes_.center, target_mes_.radius);
            
            if (vis_score > ctx_->visibility_threshold_) {
                ROS_INFO("VisPRM: Found existing roadmap node %lu with sufficient visibility (Score: %.2f).", i, vis_score);

                goal_ids.push_back(i);

                // Attempt to find path from Start -> This Node
                std::vector<VertexDesc> path_idx = graph_.shortestPath(root_id_, i);
                if (!path_idx.empty()) {
                    ROS_INFO("VisPRM: Path to existing visible node found!");
                    finalizePath(root_id_, i);
                    return true;
                }
            }
        }

        int goal_count = 0;
        int max_iterations = 1000; // Outer loop limit to avoid infinite run
        for (int iter = 0; iter < max_iterations; ++iter) {
            // ROS_INFO("Number of connected components in roadmap is: %d", graph_.countConnectedComponents(10));
            // ROS_INFO("Number of nodes in roadmap is: %zu", graph_.getNumVertices());

            double elapsed = (ros::WallTime::now() - start_time).toSec();
            if (elapsed > time_cap_) {
                ROS_WARN("VisPRM: Time cap of %d s reached (elapsed: %.2f s). Aborting.", time_cap_, elapsed);
                graph_.exportCCMap();
                return false;
            }

            // 2.1 Sample Goal Configuration
            if ((goal_count < prm_params_.max_goals) || ( graph_.getNumVertices() >= prm_params_.max_size)) {
                // ROS_INFO("Looking for more goal configurations...");
		        // ROS_INFO("goal count is %d but max_goals is %d so looking for more goal configurations", goal_count, prm_params_.max_goals);

                bool found_goal = false;
                std::vector<double> q_goal;

                if (use_visibility_integrity_ ) {
                    if (sampleVisibilityGoal(q_goal)) {
                        found_goal = true;
                        // ROS_INFO("Found a goal configuration");
                        // ROS_INFO(
                        //     "New goal configuration is (%.2f, %.2f, %.2f deg, %.2f deg, %.2f deg, %.2f deg, %.2f deg, %.2f deg, %.2f deg)",
                        //     q_goal[0],
                        //     q_goal[1],
                        //     angles::to_degrees(q_goal[2]),
                        //     angles::to_degrees(q_goal[3]),
                        //     angles::to_degrees(q_goal[4]),
                        //     angles::to_degrees(q_goal[5]),
                        //     angles::to_degrees(q_goal[6]),
                        //     angles::to_degrees(q_goal[7]),
                        //     angles::to_degrees(q_goal[8])
                        // );                    
                    }
                }

                if (found_goal) {

                    bool added = false;
                    VertexDesc g_id;
                    if (!ctx_->validity_checker_->isValid(q_goal))
                        ROS_ERROR("THE GOAL STATE IS INVALID - THIS SHOULD NOT HAPPEN");
                    // Connect to k-NN

                    g_id = connectToGraph(q_goal, prm_params_.num_neighbors);
                    if (g_id != -1) {
                        goal_ids.push_back(g_id);
                        goal_count++;
                        ROS_INFO("Connected a goal configuration to graph");
                    }


                }
            }



	    // ROS_INFO("Degree of strat vertex:");
	    // graph_.printVertexDegrees(root_id_);


            ROS_INFO("Looking for path in roadmap...");

            // 2.2 Check for Path
            for (auto g_id : goal_ids) {
                //ROS_INFO("Degree of goal vertex %zu:", g_id);
                //graph_.printVertexDegrees(g_id);
                if (graph_.inSameComponent(root_id_, g_id)) {
                    ROS_WARN("VisPRM: the start cfg %zu and goal cfg %zu are in the same connected component", root_id_, g_id);

                    // Check if start is connected to this goal
                    // We use BFS or Dijkstra. Dijkstra is already available.
                    std::vector<VertexDesc> path_idx = graph_.shortestPath(root_id_, g_id);
                    if (!path_idx.empty()) {
                        ROS_WARN("VisPRM: Found solution via visibility integrity!");
                        double elapsed = (ros::WallTime::now() - start_time).toSec();
                        ROS_WARN("VisPRM: success in %.2f seconds", elapsed);
                        finalizePath(root_id_, g_id);
                        return true;
                    }
                // ROS_WARN("VisPRM: No path exists between cfgs %zu and cfg %zu", root_id_, g_id);
                }
            }

            // 2.3 Expand Roadmap
            if (!use_visibility_integrity_ || (graph_.getNumVertices() < prm_params_.max_size)) {
                ROS_INFO("VisPRM: Expanding roadmap (Iter %d)...", iter);
                for (int i = 0; i < prm_params_.num_samples; ++i) {
                    std::vector<double> q_rand = ctx_->sampler_->sampleUniform();
                    if (ctx_->validity_checker_->isValid(q_rand)) {
                        
                        VertexDesc v_id;

                        if ((graph_.getNumVertices() < prm_params_.max_size)){
                            v_id = addState(q_rand);
                            connectToGraph(q_rand, prm_params_.num_neighbors, v_id);
                        }
                        else {
                            v_id = connectToGraph(q_rand, prm_params_.num_neighbors);
                        }
                            
                        bool perform_vis_check = true;

                        // Oracle Check & Path Termination
                        if (perform_vis_check && v_id != -1) {
                            double vis_score = ctx_->vis_oracle_->checkBallBeamVisibility(graph_.getVertexPose(v_id), target_mes_.center, target_mes_.radius);
                            
                            if (vis_score > ctx_->visibility_threshold_) {
                                ROS_INFO("VisPRM: New expanded node %lu sees target (Score: %.2f)", v_id, vis_score);
                                
                                // Check if this new visible node connects back to the start
                                std::vector<VertexDesc> path_idx = graph_.shortestPath(root_id_, v_id);
                                if (!path_idx.empty()) {
                                    ROS_WARN("VisPRM: Found solution via roadmap expansion!");
                                    double elapsed = (ros::WallTime::now() - start_time).toSec();
                                    ROS_WARN("VisPRM: success in %.2f seconds", elapsed);
                                    finalizePath(root_id_, v_id);
                                    return true;
                                }
                            }
                        }
                    }
                }
            }


            if (!start_cfg_connected) {
                ROS_WARN("Attempting to connect start configuration to expanded roadmap...");
                std::vector<double> q_start = graph_.getVertexConfig(root_id_);
                VertexDesc res_id = connectToGraph(q_start, prm_params_.num_neighbors, root_id_);
                if (res_id != -1) {
                    ROS_WARN("Start configuration is now connected");
                    start_cfg_connected = true;
                }
            }
            // 2.4 Reconnect Existing Goals to Expanded Roadmap
            // ROS_INFO("VisPRM: Attempting to connect existing goals to the expanded roadmap...");
            
            // for (auto g_id : goal_ids) {
            //     // Retrieve the joint configuration for the existing goal vertex
            //     std::vector<double> q_goal = graph_.getVertexConfig(g_id);
                
            //     // Pass the configuration, the number of neighbors, and the existing vertex ID
            //     // to prevent it from creating a duplicate state in the graph.
            //     connectToGraph(q_goal, prm_params_.num_neighbors, g_id);
            // }
        }

        ROS_WARN("VisPRM: Max iterations reached without solution.");
        return false;
    }

public:
    /**
     * @brief Grows the roadmap until every target is seen by enough reachable vertices.
     *
     * Begins by asking what the roadmap ALREADY answers: a roadmap that survived
     * earlier queries may hold vertices that see these targets, and finding them costs
     * a scan rather than a search. Then alternates between sampling a goal for a target
     * still short of its coverage and expanding the roadmap, until every target is
     * satisfied or the run is out of time.
     *
     * The graph is not reset. One planner per region is the contract, so its roadmap is
     * that region's roadmap, and reusing it across queries is what keeps the amortized
     * cost low.
     */
    bool planCoverage() override {
        if (!beginCoverageRun()) return false;

        // Restricted to the arm, VisualIK cannot move the base, so a snapped
        // configuration inherits the base of the vertex it snapped from. Restored when
        // this scope ends, whichever way it ends.
        IKGroupGuard ik_guard(ctx_->getVisualIK(), ikGroupForRun());

        root_id_ = addState(start_joint_values_);
        checked_vertices_.clear();

        size_t start_num_neighbors = (graph_.getNumVertices() > 100) ? prm_params_.num_neighbors
                                                                    : prm_params_.num_neighbors * 3;
        bool start_cfg_connected =
            (connectToGraph(start_joint_values_, start_num_neighbors, root_id_) != -1);

        ROS_INFO("VisPRM coverage: start configuration %s to roadmap of %lu vertices.",
                 start_cfg_connected ? "CONNECTED" : "NOT connected", graph_.getNumVertices());

        ros::WallTime start_time = ros::WallTime::now();

        // Scan what the roadmap already holds. The bound is read once, so vertices the
        // scan itself adds by snapping are not rescanned -- they were tested as they
        // were created.
        const size_t scanned_vertices = graph_.getNumVertices();
        for (size_t v = 0; v < scanned_vertices; ++v) {
            recordVisibleTargets(v, graph_.getVertexConfig(v), graph_.getVertexPose(v),
                                 snap_fov_on_scan_);
        }

        const int max_iterations = 1000;
        for (int iter = 0; iter < max_iterations; ++iter) {
            if (coverageComplete()) break;

            double elapsed = (ros::WallTime::now() - start_time).toSec();
            if (elapsed > time_cap_) {
                ROS_WARN("VisPRM coverage: time cap of %d s reached (elapsed: %.2f s).",
                         time_cap_, elapsed);
                break;
            }

            // --- Sample a goal for one target still short of its coverage ---
            // max_goals caps how many goal configurations a target may HOLD, not how
            // many times one is attempted: sampling goals is expensive, and past a
            // handful of them the useful work is connecting the ones already found
            // rather than finding more. Counted per target, since one shared budget
            // would be spent by whichever targets came up first.
            int t = pickUnsatisfiedTarget();
            if (t >= 0 && use_visibility_integrity_ &&
                static_cast<int>(coverage_.goals[t].size()) < prm_params_.max_goals) {

                std::vector<double> q_goal;
                if (sampleVisibilityGoal(targets_[t], q_goal) && inLocality(q_goal)) {
                    // Added whether or not it connects to anything yet: an isolated goal
                    // is a candidate the coverage count simply does not credit until the
                    // roadmap grows around it, which is what deficits are recomputed for.
                    VertexDesc g = addState(q_goal);
                    connectToGraph(q_goal, prm_params_.num_neighbors, g);
                    recordVisibleTargets(g, q_goal, graph_.getVertexPose(g),
                                         snap_fov_on_insert_);
                }
            }

            // --- Expand the roadmap ---
            if (graph_.getNumVertices() < static_cast<size_t>(prm_params_.max_size)) {
                for (int i = 0; i < prm_params_.num_samples; ++i) {
                    std::vector<double> q_rand = sampleLocalUniform();
                    if (!ctx_->validity_checker_->isValid(q_rand)) continue;

                    VertexDesc v_id = addState(q_rand);
                    connectToGraph(q_rand, prm_params_.num_neighbors, v_id);
                    recordVisibleTargets(v_id, q_rand, graph_.getVertexPose(v_id),
                                         snap_fov_on_insert_);
                }
            }

            if (!start_cfg_connected) {
                std::vector<double> q_start = graph_.getVertexConfig(root_id_);
                if (connectToGraph(q_start, prm_params_.num_neighbors, root_id_) != -1) {
                    ROS_WARN("VisPRM coverage: start configuration is now connected.");
                    start_cfg_connected = true;
                }
            }
        }

        coverageComplete();
        logCoverageSummary("VisPRM coverage", start_time);
        return coverage_.complete;
    }
};

} // namespace visual_planner
