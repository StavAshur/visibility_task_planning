#pragma once

#include "VisibilityPlannerBase.h"

namespace visual_planner {

/**
 * @brief Visibility RRT.
 *
 * Grows a tree from the start configuration, biasing a fraction of its samples toward
 * configurations that see the target (when visibility guidance is enabled), and
 * finishes as soon as a node sees enough of the target ball. Optionally snaps a node
 * whose *position* already sees the target onto a properly oriented configuration via
 * VisualIK, which is what distinguishes VisRRT from a plain RRT.
 *
 * Intended as a base for VisRRT variants; the primitives it is written in terms of
 * live in VisibilityPlannerBase.
 */
class VisRRTPlanner : public VisibilityPlannerBase {
public:
    explicit VisRRTPlanner(std::shared_ptr<PlanningContext> ctx, bool snap_fov_on_insert = true)
        : VisibilityPlannerBase(std::move(ctx))
    {
        snap_fov_on_insert_ = snap_fov_on_insert;
    }

    bool plan() override
    {
        // 0. Initialize
        reset();

        // Use member variable start_joint_values_
        if (start_joint_values_.empty()) {
            ROS_ERROR("Start joint values not set!");
            return false;
        }

        if (!ctx_->validity_checker_->isValid(start_joint_values_)) {
            ROS_ERROR("Start state is invalid!");
            for (size_t i = 0; i < start_joint_values_.size(); ++i) {
                ROS_ERROR("  Joint %lu: %f", i, start_joint_values_[i]);
            }
	    return false;
        }
        
        root_id_ = addState(start_joint_values_);

        // Calculate initial sampling radius
        Eigen::Vector3d start_ee = solveFK(start_joint_values_).translation();
        
        double sampling_radius = (target_mes_.center - start_ee).norm() * 0.5;
        
        std::uniform_real_distribution<double> dist_01(0.0, 1.0);

        ROS_INFO("Starting VisRRT...");
        ROS_INFO("visibility threshold is %f", ctx_->visibility_threshold_);
        ros::WallTime start_time = ros::WallTime::now();
        // 1. Main Loop
        for (int i = 0; i < rrt_params_.max_iterations; ++i) {
            
            double elapsed = (ros::WallTime::now() - start_time).toSec();
            if (elapsed > time_cap_) {
                ROS_WARN("VisRRT: Time cap of %d s reached (elapsed: %.2f s). Aborting.", time_cap_, elapsed);
                return false;
            }

            if (i % 100 == 0)
                ROS_INFO("Starting iteration %d", i);

            std::vector<double> q_rand;
            bool valid_sample = false;

            // --- Step 1: Sampling ---
            if (dist_01(rng_) > rrt_params_.goal_bias) {
                // Uniform
                q_rand = ctx_->sampler_->sampleUniform();
                valid_sample = true;
            } 
            else {
                if (use_visibility_integrity_) {
                    if (sampleVisibilityGoal(q_rand)) {
                        valid_sample = true;
                    }
                }
            }

            if (!valid_sample) {
                q_rand = ctx_->sampler_->sampleUniform();
            }

            // --- Step 2: Nearest Neighbor ---
            std::vector<VertexDesc> nearest = nn_.kNearest(q_rand, 1);
            if (nearest.empty()) continue;
            VertexDesc q_near_id = nearest[0];
            std::vector<double> q_near = graph_.getVertexConfig(q_near_id);

            // --- Step 3: Extend ---
            std::vector<double> q_new = extend(q_near, q_rand);

                // --- Step 4: Add to Tree ---
            if (distance(q_near, q_new) < 1e-4) continue; 
            
            VertexDesc q_new_id = addState(q_new, false);
            if (q_new_id == -1) continue;

            graph_.addEdge(q_near_id, q_new_id, distance(q_near, q_new));

            // --- Step 5: Check Goal / Visual IK Connection ---
            
            // Check A: Direct Visibility
            double current_vis = ctx_->vis_oracle_->checkBallBeamVisibility(q_new, target_mes_.center, target_mes_.radius);
            
            if (current_vis > ctx_->visibility_threshold_) {
                ROS_WARN("VisRRT: Found solution via direct visibility.");
                double elapsed = (ros::WallTime::now() - start_time).toSec();
                ROS_WARN("VisRRT: success in %.2f seconds", elapsed);
                std::vector<VertexDesc> path_idx = graph_.shortestPath(root_id_, q_new_id);
                result_path_.clear();
                finalizePath(root_id_, q_new_id);
                return true;
            }

            // Check B: VisualIK Variant
            if (snap_fov_on_insert_) {
                // Get current position
                Eigen::Vector3d current_pos = solveFK(q_new).translation();

                // Check if this *position* is good
                double potential_vis = ctx_->vis_oracle_->checkBallBeamVisibility(
                    current_pos, 
                    target_mes_.center,
                    target_mes_.radius
                );

                if (potential_vis > ctx_->visibility_threshold_) {
                    // ROS_INFO("VisRRT: Using VisIK to extend to goal");
                    // Try to snap
                    Eigen::Matrix3d look_at_rot = ctx_->sampler_->computeLookAtRotation(current_pos, target_mes_.center);

                    std::vector<double> q_snapped;

                    if (ctx_->vis_ik_->solveVisualIK(q_new, target_mes_, look_at_rot, q_snapped)) {
                        // VisualIK places the tool where the target ball fits inside the
                        // beam, but picks that point from geometry alone: it never
                        // consults the obstacles. The check above answered for THIS
                        // node's position, and the snapped pose can sit further from the
                        // target than the node does, where an obstacle may occlude it.
                        // Asking the oracle is what keeps a success from meaning
                        // anything less than "this configuration sees the target".
                        // Tested before the edge, being the cheaper of the two.
                        double snapped_vis = ctx_->vis_oracle_->checkBallBeamVisibility(
                            q_snapped, target_mes_.center, target_mes_.radius);

                        if (snapped_vis > ctx_->visibility_threshold_ && validateEdge(q_new, q_snapped)) {
                            VertexDesc goal_id = addState(q_snapped, false);
                            if (goal_id != -1) {
                                graph_.addEdge(q_new_id, goal_id, distance(q_new, q_snapped));
                                
                                ROS_WARN("VisRRT: Found solution via VisualIK snap.");
                                double elapsed = (ros::WallTime::now() - start_time).toSec();
                                ROS_WARN("VisRRT: success in %.2f seconds", elapsed);
                                std::vector<VertexDesc> path_idx = graph_.shortestPath(root_id_, goal_id);
                                result_path_.clear();
                                finalizePath(root_id_, goal_id);
                                return true;
                            }
                        }
                    }
                }
            }
        }
        
        ROS_WARN("VisRRT: Max iterations reached without solution.");
        return false;
    }

    /**
     * @brief Every vertex is reachable from the root.
     *
     * The tree starts at root_id_ and every vertex afterwards is attached by an edge to
     * one already in it, snapped goals included. Reachability is therefore structural,
     * and the base class's per-goal breadth-first search would spend an O(V+E) pass per
     * iteration rediscovering it.
     */
    bool isReachable(VertexDesc) override { return true; }

    /**
     * @brief Grows one tree until every target is seen by enough of its configurations.
     *
     * The same search as plan(), asked a different question. Three differences: a
     * goal-biased sample is drawn for one target still short of its coverage rather
     * than for the single MES; every new node is tested against every unsatisfied
     * target instead of one; and the tree keeps growing after a target is satisfied,
     * stopping only when all of them are.
     */
    bool planCoverage() override
    {
        reset();

        if (!beginCoverageRun()) return false;

        // Restricted to the arm, VisualIK cannot move the base, so a snapped
        // configuration inherits the base of the node it snapped from -- already inside
        // the disk. Restored when this scope ends, whichever way it ends.
        IKGroupGuard ik_guard(ctx_->getVisualIK(), ikGroupForRun());

        root_id_ = addState(start_joint_values_);
        recordVisibleTargets(root_id_, start_joint_values_,
                             graph_.getVertexPose(root_id_), snap_fov_on_insert_);

        ros::WallTime start_time = ros::WallTime::now();

        for (int i = 0; i < rrt_params_.max_iterations; ++i) {
            if (coverageComplete()) break;

            double elapsed = (ros::WallTime::now() - start_time).toSec();
            if (elapsed > time_cap_) {
                ROS_WARN("VisRRT coverage: time cap of %d s reached (elapsed: %.2f s).",
                         time_cap_, elapsed);
                break;
            }

            if (i % 100 == 0) {
                ROS_INFO("VisRRT coverage: iteration %d", i);
            }

            // --- Step 1: Sampling ---
            // UNIFORM ONLY. The goal-biased draw that used to sit here aimed at one
            // unsatisfied target, drawn uniformly, and asked the VI-tree for a position
            // that sees it (sampleVisibilityGoal). Removed 2026-09-16 at Stav's
            // request: it did not pay for itself here. Note what follows from that --
            // NOTHING in this query picks a target any more. Targets are no longer
            // chased one at a time; every node inserted is tested against EVERY target
            // still short of coverage, in recordVisibleTargets(). plan(), which serves
            // the paper's single-target query, keeps its goal bias.
            std::vector<double> q_rand = sampleLocalUniform();

            // --- Step 2: Nearest Neighbor ---
            std::vector<VertexDesc> nearest = nn_.kNearest(q_rand, 1);
            if (nearest.empty()) continue;
            VertexDesc q_near_id = nearest[0];
            std::vector<double> q_near = graph_.getVertexConfig(q_near_id);

            // --- Step 3: Extend ---
            std::vector<double> q_new = extend(q_near, q_rand);
            if (distance(q_near, q_new) < 1e-4) continue;

            // --- Step 4: Add to Tree ---
            // The pose is stored rather than discarded: testing n targets against this
            // node would otherwise repeat the same forward kinematics n times.
            VertexDesc q_new_id = addState(q_new);
            if (q_new_id == -1) continue;
            graph_.addEdge(q_near_id, q_new_id, distance(q_near, q_new));

            // --- Step 5: Credit whatever this node can see ---
            recordVisibleTargets(q_new_id, q_new, graph_.getVertexPose(q_new_id),
                                 snap_fov_on_insert_);
        }

        coverageComplete();
        logCoverageSummary("VisRRT coverage", start_time);
        return coverage_.complete;
    }
};

} // namespace visual_planner
