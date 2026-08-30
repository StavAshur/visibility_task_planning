#pragma once

#include <vector>
#include <set>
#include <algorithm>
#include <cmath>
#include <memory>
#include <iostream>
#include <fstream>
#include <unordered_set>
#include <unordered_map>
#include <random>
#include <map>
#include <queue>
#include <numeric>

#include <Eigen/Core>
#include <Eigen/Geometry>

// Dependencies
#include "VisibilityOracle.h"
#include "NearestNeighbor.h"
#include "Graph.h"
#include "../common/Types.h" 
#include "../components/Sampler.h"

namespace visual_planner {
//
struct VINode {
    int id;                 // Unique ID
    BoundingBox box;        // The bounding box
    std::string path_idx;   // "0", "01", etc. (for debugging)
    
    std::unique_ptr<VINode> left = nullptr;
    std::unique_ptr<VINode> right = nullptr;
    VINode* parent = nullptr;
    
    
    bool is_leaf = false;
    int index = -1;    // If leaf, index in the graph

    int height = 0; // 0 for leaves, 1+ for internal
    std::vector<std::pair<VINode*, double>> visible_from_nodes; // IDs of nodes this node sees + visibility score
    double convexity_score = 0.0;
    bool visibility_computed = false;
};

/**
 * @brief Result of a multi-target VI-tree query (VisTSP subproblem 1).
 *
 * Holds, for a vector of target balls, which workspace regions see a large enough
 * part of each target. The two members are the same relation in both directions:
 * region pruning walks regions and asks which targets it would lose, then validates
 * per target that each is still covered enough times, so both are built once here.
 */
struct MultiTargetQueryResult {
    /// per_target[t] = the regions seeing at least a beta-fraction of target t,
    /// each paired with that fraction (in [0, 1]). Indexed as the query's input.
    std::vector<std::vector<std::pair<VINode*, double>>> per_target;

    /// The inverse: for each region that survived the beta threshold for at least
    /// one target, the indices of the targets it sees.
    std::unordered_map<VINode*, std::vector<int>> region_to_targets;
};

class VisibilityIntegrity {
public:
    VisibilityIntegrityParams vi_params_;
    VisibilityToolParams vis_tool_params_;
    
    VisibilityIntegrity() 
        : vi_params_(), vis_tool_params_(), rng_(std::random_device{}()) 
    {
        // Default bounds
        workspace_bounds_ = {-2.0, 2.0, -2.0, 2.0, 0.0, 2.0};
    }

    void setVisibilityOracle(std::shared_ptr<VisibilityOracle> oracle) {
        vis_oracle_ = oracle;
    }
    
    void setSampler(std::shared_ptr<Sampler> sampler) {
        sampler_ = sampler;
    }

    void setVIParams(const VisibilityIntegrityParams& vi_params) {
        vi_params_ = vi_params;
    }

    void setVTParams(const VisibilityToolParams& vis_tool_params) {
        vis_tool_params_ = vis_tool_params;
    }

    void setWorkspaceBounds(const BoundingBox& bounds) {
        workspace_bounds_ = bounds;
    }

    const BoundingBox& getWorkspaceBounds() const {
        return workspace_bounds_;
    }

    /**
     * @brief Main initialization function.
     */
    void build() {
        if (!vis_oracle_ || !sampler_) {
            std::cerr << "[VisibilityIntegrity] Error: Oracle or Sampler not set." << std::endl;
            return;
        }
        ros::WallTime start_time = ros::WallTime::now();

        std::cout << "[VisibilityIntegrity] Building Visibility Integrity Tree..." << std::endl;
        
        // Drop everything the previous tree left behind before its nodes are freed:
        // all of this state indexes into nodes that are about to stop existing.
        leaves_.clear();
        touched_.clear();
        accum_.clear();
        num_nodes_ = 0;

        int node_counter = 0;
        int leaf_counter = 0;

        // Initial Convexity Check
        // Use Sampler to get obstacle-free points in the whole workspace
        std::vector<Eigen::Vector3d> S;
        sampler_->sampleInBox(workspace_bounds_, vi_params_.num_samples, S);
        
        root_ = std::make_unique<VINode>();
        root_->box = workspace_bounds_;
        root_->path_idx = "";
        root_->id = node_counter++;

        buildTreeRecursive(root_.get(), workspace_bounds_, "0", node_counter, leaf_counter);

        // Ids occupy [0, node_counter) exactly; size the query accumulator to match.
        num_nodes_ = static_cast<size_t>(node_counter);
        accum_.assign(num_nodes_, 0.0);
        touched_.clear();

        ComputeLeafPairwiseVisibility();
        ComputeParentVisibilityBottomUp(root_.get());

        double elapsed = (ros::WallTime::now() - start_time).toSec();

        ROS_WARN("[VisibilityIntegrity] Tree built in %f seconds. Number of leaves: %zu", elapsed, leaves_.size());

    }



/// ========================================================================================================
/// ========================================================================================================
/// ============================================== Queries =================================================
/// ========================================================================================================
/// ========================================================================================================

    /**
     * @brief Samples a point that can see the target 'ball' with high probability.
     * Uses the "visible_from_nodes" list (Inverse Visibility) to find promising candidates.
     */
    bool SampleFromVisibilityRegion(const Ball& ball, Eigen::Vector3d& sampled_point, double visibility_threshold) {
        // 1. Find the leaf that contains the target Ball's center
        std::vector<std::pair<VINode*, double>> seers;
        query(ball, seers);

        if (seers.empty()) {
            // ROS_WARN("[SampleFromVisibilityRegion] no one sees this point????");
            return false;
        }

        // ROS_INFO("[SampleFromVisibilityRegion] Query success! Found %d seers.", (int)seers.size());
        // for (size_t i = 0; i < seers.size(); ++i) {
        //     ROS_INFO("  -> Seer [%d]: Node ID %d | Intersection Vol: %.4f", 
        //              (int)i, 
        //              seers[i].first->id, 
        //              seers[i].second);
        // }

        // 2. Rejection Sampling Loop
        int max_attempts = 1000;
        std::uniform_int_distribution<> dist_idx(0, seers.size() - 1);
        std::uniform_real_distribution<double> dist_prob(0.0, 1.0);

        // The scores returned by query() are fractions of the ball in [0, 1], and
        // are typically far below 1 (a seer rarely covers the whole target). Using
        // them directly as acceptance probabilities would reject almost everything,
        // so rescale by the best score: this preserves the relative ranking of the
        // seers -- all rejection sampling needs here -- while keeping the acceptance
        // rate of the top candidate at 1.
        double max_score = 0.0;
        for (const auto& seer_pair : seers) {
            max_score = std::max(max_score, seer_pair.second);
        }
        if (max_score <= 1e-9) return false; // No seer covers a meaningful part of the ball

        for (int i = 0; i < max_attempts; ++i) {
            // A. Pick a random candidate node from the list
            int rand_idx = dist_idx(rng_);
            const auto& candidate_pair = seers[rand_idx];

            VINode* seer_node = candidate_pair.first;
            double relative_score = candidate_pair.second / max_score;

            // B. Rejection Sampling: Reject with probability (1 - score)
            // (i.e., Accept if random value < score)
            if (dist_prob(rng_) > relative_score) {
                continue;
            }

            // C. Sample a point from the seer's bounding box
            std::vector<Eigen::Vector3d> samples;
            // sampleInBox ensures the point is within bounds and obstacle-free
            if (!sampler_->sampleInBox(seer_node->box, 1, samples) || samples.empty()) {
                continue; 
            }
            Eigen::Vector3d candidate_point = samples[0];

            // D. Verify actual visibility against the Ball
            // Returns fraction of ball visible from candidate_point
            double visible_fraction = vis_oracle_->checkBallBeamVisibility(candidate_point, ball);

            if (visible_fraction > visibility_threshold) {
                sampled_point = candidate_point;
                return true;
            }
        }

        return false;
    }

    /**
     * @brief Finds the leaf containing the query point and populates the input vector
     * with 'visible_from_nodes' from all ancestors along the path.
     * * @param query_point The point to search for.
     * @param out_seeing_nodes Output vector to populate with (node_id, score) pairs.
     * @return int The leaf_index of the found leaf, or -1 if not found.
     */
    VINode* query(const Eigen::Vector3d& query_point, std::vector<std::pair<VINode*, double>>& out_seeing_nodes) {
        // Clear previous results to ensure the vector contains only relevant nodes
        out_seeing_nodes.clear();

        if (!root_) return nullptr;

        // Helper lambda: Check if point is inside a box
        auto contains = [](const BoundingBox& b, const Eigen::Vector3d& p) {
            return (p.x() >= b.x_min && p.x() <= b.x_max &&
                    p.y() >= b.y_min && p.y() <= b.y_max &&
                    p.z() >= b.z_min && p.z() <= b.z_max);
        };

        // 1. Fast fail if outside the global workspace
        if (!contains(root_->box, query_point)) {
            return nullptr;
        }

        VINode* current = root_.get();

        // 2. Traverse down to the leaf
        while (current) {
            // Accumulate visibility from the current node on the path
            out_seeing_nodes.insert(
                out_seeing_nodes.end(),
                current->visible_from_nodes.begin(),
                current->visible_from_nodes.end()
            );

            if (current->is_leaf) {
                return current;
            }

            // Determine which child to traverse
            if (current->left && contains(current->left->box, query_point)) {
                current = current->left.get();
            } 
            else if (current->right && contains(current->right->box, query_point)) {
                current = current->right.get();
            } 
            else {
                // Point is within parent box but not strictly in children (gap/precision).
                return nullptr; 
            }
        }

        return nullptr;
    }

/**
     * @brief Populates 'out_seeing_nodes' with (Seer, BallFraction) pairs.
     *
     * Traverses the tree collecting the nodes that cover the ball: those fully
     * contained in it (recursion stops there) and the leaves that intersect it
     * partially. This cover is disjoint, so each seer's fractions can be summed.
     *
     * Every covering node contributes getBallFraction(node->box, ball) -- the
     * fraction of the ball's volume it accounts for -- to each node that sees it.
     * The accumulated score per seer is therefore a dimensionless value in [0, 1]:
     * the fraction of the target ball visible from that seer.
     */
    void query(const Ball& ball, std::vector<std::pair<VINode*, double>>& out_seeing_nodes) {
        out_seeing_nodes.clear();
        if (!root_) return;

        // Recursive traversal lambda
        std::function<void(VINode*)> traverse = [&](VINode* node) {
            if (!node) return;
            // ROS_INFO("[BallQuery] Visiting Node ID: %d | Path: %s | Box: [%.2f, %.2f] [%.2f, %.2f] [%.2f, %.2f]", 
            //          node->id, 
            //          node->path_idx.c_str(),
            //          node->box.x_min, node->box.x_max,
            //          node->box.y_min, node->box.y_max,
            //          node->box.z_min, node->box.z_max);
            // 1. Check Intersection (Pruning)
            // If the box is completely outside the ball radius, skip it.
            double d2 = distSqPointBox(ball.center, node->box);
            if (d2 > ball.radius * ball.radius) {
                // ROS_INFO("   -> Pruned: Box is outside ball radius.");
                return; 
            }

            bool fully_contained = isBoxFullyInBall(node->box, ball);
            // if (fully_contained) ROS_INFO("   -> Status: Fully Contained.");
            // else if (node->is_leaf) ROS_INFO("   -> Status: Leaf Intersecting.");
            // else ROS_INFO("   -> Status: Partial Intersection (Internal).");
            // 2. Process "Relevant" Nodes (Fully Contained OR Leaves)
            if (fully_contained || node->is_leaf) {
                // Fraction of the ball covered by this node. getBallFraction
                // handles the fully-contained and the partial case alike, so both
                // branches yield comparable, dimensionless values.
                double ball_fraction = getBallFraction(node->box, ball);

                // If the fraction is non-negligible, process visibility
                if (ball_fraction > 1e-9) {
                    // ROS_INFO("   -> Processing %lu seers from this node (Frac: %.4f):",
                    //          node->visible_from_nodes.size(), ball_fraction);
                    // Iterate over 'visible_from_nodes' (which stores pairs <VINode*, double>)
                    // and credit this covering node's fraction to every seer of it.
                    for (const auto& seer_pair : node->visible_from_nodes) {
                        VINode* u = seer_pair.first;

                        // A zero entry means "not yet credited in this query": ball_fraction
                        // is strictly positive here, so a credited entry never returns to 0.
                        if (accum_[u->id] == 0.0) touched_.push_back(u);
                        accum_[u->id] += ball_fraction;
                    }
                }

                // If fully contained, children are also fully contained.
                // We stop here to avoid double counting volume (assuming disjoint partition strategy)
                // or simply because the prompt says "whenever we find a node... fully contained".
                if (fully_contained) return;
            }

            // 3. Recurse (Internal nodes that are NOT fully contained)
            if (!node->is_leaf) {
                traverse(node->left.get());
                traverse(node->right.get());
            }
        };

        traverse(root_.get());

        // Emit the accumulated seers and restore the all-zeros invariant on accum_.
        out_seeing_nodes.reserve(touched_.size());
        for (VINode* u : touched_) {
            out_seeing_nodes.push_back({u, accum_[u->id]});
            accum_[u->id] = 0.0;
        }
        touched_.clear();
    }

    /**
     * @brief Multi-target query: which regions see enough of each of several targets.
     *
     * For every target ball, collects the regions that see at least a `threshold`
     * fraction of it. This is the entry point of the VisTSP pipeline: the regions are
     * what region pruning thins down, what the base-location set cover must cover, and
     * where the local planner looks for configurations that view a target.
     *
     * The per-target fractions are accumulated independently. They may only be summed
     * within a single target, where the covering nodes form a disjoint cover of that
     * ball; fractions for different targets measure different volumes and are not
     * comparable as a sum.
     *
     * @param targets   The target balls to cover.
     * @param out       Populated with the region/target relation in both directions.
     *                  Any previous contents are discarded.
     * @param threshold Coverage fraction in [0, 1] (the beta of the VisTSP notes). A
     *                  region is reported for a target when it sees at least this
     *                  fraction of that target's ball. A target may end up with no
     *                  regions at all if the threshold exceeds what any single region
     *                  achieves for it.
     */
    void query(const std::vector<Ball>& targets, MultiTargetQueryResult& out,
               double threshold = 0.8) {
        out.per_target.clear();
        out.region_to_targets.clear();
        out.per_target.resize(targets.size());

        if (!root_) return;

        // Reused across targets so the per-target seer list costs no fresh allocation.
        std::vector<std::pair<VINode*, double>> seers;

        for (size_t t = 0; t < targets.size(); ++t) {
            query(targets[t], seers);

            for (const auto& seer_pair : seers) {
                if (seer_pair.second >= threshold) {
                    out.per_target[t].push_back(seer_pair);
                    out.region_to_targets[seer_pair.first].push_back(static_cast<int>(t));
                }
            }

            if (out.per_target[t].empty()) {
                ROS_WARN("[VisibilityIntegrity] Multi-target query: target %zu has no region "
                         "seeing a %.3f fraction of it (%zu regions see it at all).",
                         t, threshold, seers.size());
            }
        }
    }



    bool isConnectedToTarget(const Eigen::Vector3d& query_point, const Eigen::Vector3d& target_point) {
        return true;
    }

    /// Whether build() has produced a tree. Every query answers emptily until it has,
    /// which a caller cannot otherwise distinguish from "nothing sees the target".
    bool isBuilt() const { return root_ != nullptr; }


private:
    std::shared_ptr<VisibilityOracle> vis_oracle_;
    std::shared_ptr<Sampler> sampler_;
    std::mt19937 rng_;
    BoundingBox workspace_bounds_;
    size_t num_nodes_ = 0;
    std::unique_ptr<VINode> root_;
    std::vector<VINode*> leaves_;

    // Sparse accumulator for query(Ball, ...): sums each seer's ball fractions.
    // Node ids are dense (0 .. num_nodes_-1), so accum_ is indexed directly by id.
    // INVARIANT: accum_ is all zeros between queries -- a query resets only the entries
    // listed in touched_, keeping its cost proportional to the seers actually hit.
    std::vector<double> accum_;      // accum_[id] = ball fraction accumulated for that node
    std::vector<VINode*> touched_;   // the nodes with a currently-nonzero accum_ entry



/// ========================================================================================================
/// ========================================================================================================
/// ========================================= Tree construction ============================================
/// ========================================================================================================
/// ========================================================================================================


    
    bool buildTreeRecursive(VINode* node, const BoundingBox& env_bounds, const std::string& idx, int& node_counter, int& leaf_counter) {
        
        // Calculate size early for logging
        Eigen::Vector3d box_size = getBoxSize(node->box);

        // ROS_INFO("[Node %s] Processing ID: %d | Bounds: [%.2f, %.2f, %.2f]", 
        //         idx.c_str(), node_counter, box_size.x(), box_size.y(), box_size.z());

        // 1. Sample S_in (Valid points inside current box)
        std::vector<Eigen::Vector3d> S_in;

        // Decreasing the number of samples for S_in as the box volume decreases 
        int in_samples_num = std::min(static_cast<double>(vi_params_.num_samples), getBoxVolume(node->box)*30*30*30);
        sampler_->sampleInBox(node->box, in_samples_num, S_in);

        // 2. Sample S_out (Valid points in Env \ box)
        std::vector<Eigen::Vector3d> S_out;
        if (node_counter == 1) {
            // Root node logic
            sampler_->sampleInBox(node->box, vi_params_.num_samples, S_out);
        } else {
            sampler_->sampleOutside(env_bounds, node->box, vi_params_.num_samples, S_out, vis_tool_params_.beam_length);
        }

        // DEBUG: Check sample sizes (casting size_t to unsigned long for %lu)
        // ROS_INFO("Samples: S_in=%lu, S_out=%lu", S_in.size(), S_out.size());

        if (S_in.empty() || S_out.empty()) {
            // ROS_WARN("[Node %s] EMPTY SAMPLES! Forcing LEAF. (S_in=%lu, S_out=%lu)", 
            //         idx.c_str(), S_in.size(), S_out.size());
            return false;
        }

        // 3. Compute Score (Intersection of visibility) - GPU OPTIMIZED
        
        // Combine sets to create a single context for the NxN matrix
        std::vector<Eigen::Vector3d> combined = S_in;
        combined.insert(combined.end(), S_out.begin(), S_out.end());

        // A. Upload and compute NxN matrix on GPU
        vis_oracle_->precomputeGpuMatrix(combined);

        // B. Define "Guards" (S_in)
        std::vector<int> guard_indices(S_in.size());
        std::iota(guard_indices.begin(), guard_indices.end(), 0); 

        // C. Run Reduction Kernel on GPU
        std::vector<bool> seen_by_all = vis_oracle_->checkSeenByAllGPU(guard_indices);
        std::vector<bool> seen_by_any = vis_oracle_->checkSeenByAnyGPU(guard_indices);

        // D. Count
        int numerator_count = 0;
        int denominator_count = 0;
        
        // Iterate only over the S_out portion of the combined vector
        for (size_t k = S_in.size(); k < combined.size(); ++k) {
            if (seen_by_all[k]) numerator_count++;
            if (seen_by_any[k]) denominator_count++;
        }

        double score = (denominator_count > 0) 
                    ? static_cast<double>(numerator_count) / static_cast<double>(denominator_count)
                    : 0.0;


        // E. Compute Convexity Score
        // We reuse the 'seen_by_all' vector. Indices 0 to S_in.size()-1 correspond 
        // to points in S_in checking visibility against all other S_in points.
        int convex_count = 0;
        size_t n_in = S_in.size();

        for (size_t k = 0; k < n_in; ++k) {
            if (seen_by_all[k]) {
                convex_count++;
            }
        }

        double convexity_score = (n_in > 0) 
                               ? static_cast<double>(convex_count) / static_cast<double>(n_in) 
                               : 0.0;

        node->convexity_score = convexity_score;


        // DEBUG: Print Score Details
        // ROS_INFO("Score Calc: %d/%d | Score=%.4f (Thresh: %.4f)", 
        //         numerator_count, denominator_count, score, vi_params_.vi_threshold);

        // Termination Criteria
        // Note: We used box_size calculated at the top
        double min_dim = 0.1; 
        bool too_small = (box_size.maxCoeff() < min_dim);

        // if (too_small) {
        //     ROS_WARN("Box too small! MaxDim=%.4f < %.4f", 
        //             box_size.maxCoeff(), min_dim);
        // }

        if (score > vi_params_.vi_threshold || too_small) {
        //     ROS_INFO("[Node %s] DECISION: LEAF (Final ID: %d)",idx.c_str(), node->id);
            makeLeaf(node);
            leaf_counter++;
        } else {
            // Split (Alg 2 Lines 12-16)
            int d = 0; // 0=x, 1=y, 2=z
            // Using box_size variable created at top
            if (box_size.y() >= box_size.x() && box_size.y() >= box_size.z()) d = 1;
            if (box_size.z() >= box_size.x() && box_size.z() >= box_size.y()) d = 2;

            // std::string axis_name = (d==0 ? "X" : (d==1 ? "Y" : "Z"));
            // ROS_INFO("[Node %s] DECISION: SPLIT on %s-axis", idx.c_str(), axis_name.c_str());

            BoundingBox left_box = node->box;
            BoundingBox right_box = node->box;

            if (d == 0) {
                double mid = (node->box.x_min + node->box.x_max) / 2.0;
                left_box.x_max = mid;
                right_box.x_min = mid;
            } else if (d == 1) {
                double mid = (node->box.y_min + node->box.y_max) / 2.0;
                left_box.y_max = mid;
                right_box.y_min = mid;
            } else {
                double mid = (node->box.z_min + node->box.z_max) / 2.0;
                left_box.z_max = mid;
                right_box.z_min = mid;
            }

            node->left = std::make_unique<VINode>();
            node->left->box = left_box;
            node->left->path_idx = idx + "0";
            node->left->id = node_counter++;
            node->left->parent = node;


            node->right = std::make_unique<VINode>();
            node->right->box = right_box;
            node->right->path_idx = idx + "1";
            node->right->id = node_counter++;
            node->right->parent = node;

            // Recursive calls
            bool left_success = buildTreeRecursive(node->left.get(), env_bounds, idx + "0", node_counter, leaf_counter);
            bool right_success = buildTreeRecursive(node->right.get(), env_bounds, idx + "1", node_counter, leaf_counter);

            node->height = std::max(node->left->height, node->right->height) + 1;

            if (!left_success)
                node->left = nullptr;
            if (!right_success)
                node->right = nullptr;
            if ((!left_success) && (!right_success)) {
                makeLeaf(node);            
                leaf_counter++;
            }
            
            // ROS_INFO("[Node %s] Finished processing children. Height set to %d", idx.c_str(), node->height);
        }
        return true;
    }

    void makeLeaf(VINode* node) {
        node->is_leaf = true;
        node->height = 0;
        leaves_.push_back(node);
        std::ofstream leaf_file("leaf_boxes.txt", std::ios::app);
        if (leaf_file.is_open()) {
            // Format: ID min_x max_x min_y max_y min_z max_z
            leaf_file << node->id << " "
                      << node->box.x_min << " " << node->box.x_max << " "
                      << node->box.y_min << " " << node->box.y_max << " "
                      << node->box.z_min << " " << node->box.z_max << "\n";
            leaf_file.close();
        }
    }




    /**
     * @brief Computes all-pairs visibility between leaf centers.
     * Updates 'visible_from_nodes' based on the sphere intersection constraints.
     */
    void ComputeLeafPairwiseVisibility() {
        if (leaves_.empty()) return;

        std::cout << "[VisibilityIntegrity] Computing Leaf Pairwise Visibility (" 
                  << leaves_.size() << " leaves)..." << std::endl;

        // 1. Prepare Data
        std::vector<Eigen::Vector3d> leaf_centers;
        leaf_centers.reserve(leaves_.size());
        
        for (VINode* leaf : leaves_) {
            Eigen::Vector3d c(
                (leaf->box.x_min + leaf->box.x_max) / 2.0,
                (leaf->box.y_min + leaf->box.y_max) / 2.0,
                (leaf->box.z_min + leaf->box.z_max) / 2.0
            );
            leaf_centers.push_back(c);
        }

        // 2. Run GPU Batch Visibility (All Pairs)
        // results[i * N + j] is true if leaf[i] sees leaf[j]
        std::vector<bool> visibility_matrix;
        vis_oracle_->computeBatchVisibility(leaf_centers, visibility_matrix);

        size_t N = leaves_.size();

        // 3. Update Lists
        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < N; ++j) {
                if (i == j) continue; // Skip self

                // Check visibility (Symmetric in theory, but matrix might have 1-way if P1!=P2)
                if (visibility_matrix[i * N + j]) {
                    VINode* u = leaves_[i]; // The "Seer"? Or "Target"?
                    VINode* v = leaves_[j];
                    
                    bool u_in_box = intersectsWorkspaceBox(u->box);
                    bool v_in_box = intersectsWorkspaceBox(v->box);
                    double score = 1.0; // Base score for raw visibility

                    // Update u's list with v
                    if (v_in_box) {
                        u->visible_from_nodes.push_back({v, score});
                    }

                }
            }
        }
    }

    /**
     * @brief Bottom-Up Recursive Traversal.
     * Computes internal node visibility by intersecting children's lists.
     * Propagates parent visibility back to the 'seers'.
     * Handles single-child nodes by copying and propagating.
     */
    void ComputeParentVisibilityBottomUp(VINode* node) {
        if (!node || node->is_leaf) return;

        // 1. Recurse First (Post-Order)
        if (node->left) ComputeParentVisibilityBottomUp(node->left.get());
        if (node->right) ComputeParentVisibilityBottomUp(node->right.get());

        // 2. Case A: Two Children - Intersect
        if (node->left && node->right) {
            
            // Sort lists by Node ID to enable linear intersection
            auto& list_l = node->left->visible_from_nodes;
            auto& list_r = node->right->visible_from_nodes;

            std::sort(list_l.begin(), list_l.end(), [](const auto& a, const auto& b){
                return a.first->id < b.first->id;
            });
            std::sort(list_r.begin(), list_r.end(), [](const auto& a, const auto& b){
                return a.first->id < b.first->id;
            });

            // Perform Intersection
            auto it_l = list_l.begin();
            auto it_r = list_r.begin();

            while (it_l != list_l.end() && it_r != list_r.end()) {
                if (it_l->first->id < it_r->first->id) {
                    ++it_l;
                } else if (it_r->first->id < it_l->first->id) {
                    ++it_r;
                } else {
                    // Match Found! Same Node 'S' sees both Left and Right
                    VINode* seer = it_l->first;
                    double avg_score = (it_l->second + it_r->second) / 2.0;

                    if(intersectsWorkspaceBox(seer->box)) 
                       node->visible_from_nodes.push_back({seer, avg_score});
                    
                    if(intersectsWorkspaceBox(node->box))
                        seer->visible_from_nodes.push_back({node, avg_score});

                    ++it_l;
                    ++it_r;
                }
            }
        }
        // 3. Case B: Single Child (Left or Right)
        else if (node->left || node->right) {
            VINode* child = node->left ? node->left.get() : node->right.get();
            
            // Copy the child's list to the parent
            node->visible_from_nodes = child->visible_from_nodes;

            // Update the nodes in that list by inserting the parent
            for (auto& pair : node->visible_from_nodes) {
                VINode* seer = pair.first;
                double score = pair.second;
                
                // Add parent to the seer's list
                seer->visible_from_nodes.push_back({node, score});
            }
        }
    }







/// ========================================================================================================
/// ========================================================================================================
/// ========================================= Geometric helpers ============================================
/// ========================================================================================================
/// ========================================================================================================





    bool intersectsWorkspaceSphere(BoundingBox& b){
        double cx = 0.33, cy = 0.0, cz = 0.33;
        // double cx = 0.0, cy = 0.0, cz = 0.0;
        double r_sq = 1.2 * 1.2;
        double dist_sq = 0.0;

        // Calculate squared distance from Center to the Box
        if (cx < b.x_min) dist_sq += (cx - b.x_min) * (cx - b.x_min);
        else if (cx > b.x_max) dist_sq += (cx - b.x_max) * (cx - b.x_max);

        if (cy < b.y_min) dist_sq += (cy - b.y_min) * (cy - b.y_min);
        else if (cy > b.y_max) dist_sq += (cy - b.y_max) * (cy - b.y_max);

        if (cz < b.z_min) dist_sq += (cz - b.z_min) * (cz - b.z_min);
        else if (cz > b.z_max) dist_sq += (cz - b.z_max) * (cz - b.z_max);

        return dist_sq <= r_sq;
    };


    bool intersectsWorkspaceBox(const BoundingBox& b) {
            // Goal AABB boundaries
            double goal_x_min = -5.0;
            double goal_x_max =  5.0;
            double goal_y_min = -5.0;
            double goal_y_max =  5.0;
            double goal_z_min =  0.0;
            double goal_z_max =  1.4;

            // Check for overlap along the X axis
            bool overlap_x = (goal_x_min <= b.x_max) && (goal_x_max >= b.x_min);
            
            // Check for overlap along the Y axis
            bool overlap_y = (goal_y_min <= b.y_max) && (goal_y_max >= b.y_min);
            
            // Check for overlap along the Z axis
            bool overlap_z = (goal_z_min <= b.z_max) && (goal_z_max >= b.z_min);

            // The boxes intersect if they overlap on all three axes simultaneously
            return overlap_x && overlap_y && overlap_z;
    }





    // Squared distance from a point to the closest point on a box
    double distSqPointBox(const Eigen::Vector3d& p, const BoundingBox& b) {
        double dist_sq = 0.0;
        if (p.x() < b.x_min) dist_sq += (p.x() - b.x_min) * (p.x() - b.x_min);
        else if (p.x() > b.x_max) dist_sq += (p.x() - b.x_max) * (p.x() - b.x_max);

        if (p.y() < b.y_min) dist_sq += (p.y() - b.y_min) * (p.y() - b.y_min);
        else if (p.y() > b.y_max) dist_sq += (p.y() - b.y_max) * (p.y() - b.y_max);

        if (p.z() < b.z_min) dist_sq += (p.z() - b.z_min) * (p.z() - b.z_min);
        else if (p.z() > b.z_max) dist_sq += (p.z() - b.z_max) * (p.z() - b.z_max);
        
        return dist_sq;
    }


    // Squared distance between the closest pair of points in two AABBs
    double distSqBoxBox(const BoundingBox& b1, const BoundingBox& b2) {
        // Calculate the gap on each axis. 
        // If the boxes overlap on an axis, both subtractions are negative, resulting in 0.0.
        double dx = std::max({0.0, b1.x_min - b2.x_max, b2.x_min - b1.x_max});
        double dy = std::max({0.0, b1.y_min - b2.y_max, b2.y_min - b1.y_max});
        double dz = std::max({0.0, b1.z_min - b2.z_max, b2.z_min - b1.z_max});
        
        return dx * dx + dy * dy + dz * dz;
    }

    // Checks if the Box is FULLY contained within the Ball
    bool isBoxFullyInBall(const BoundingBox& b, const Ball& ball) {
        double r_sq = ball.radius * ball.radius;
        // Check all 8 corners. If any corner is outside, the box is not fully contained.
        // (Optimization: We could check only the furthest corner, but checking 8 is robust/simpler).
        double corners[8][3] = {
            {b.x_min, b.y_min, b.z_min}, {b.x_min, b.y_min, b.z_max},
            {b.x_min, b.y_max, b.z_min}, {b.x_min, b.y_max, b.z_max},
            {b.x_max, b.y_min, b.z_min}, {b.x_max, b.y_min, b.z_max},
            {b.x_max, b.y_max, b.z_min}, {b.x_max, b.y_max, b.z_max}
        };

        for (int i = 0; i < 8; ++i) {
            double dx = corners[i][0] - ball.center.x();
            double dy = corners[i][1] - ball.center.y();
            double dz = corners[i][2] - ball.center.z();
            if (dx*dx + dy*dy + dz*dz > r_sq) return false;
        }
        return true;
    }


    Eigen::Vector3d getBoxSize(const BoundingBox& b) {
        return Eigen::Vector3d(
            b.x_max - b.x_min,
            b.y_max - b.y_min,
            b.z_max - b.z_min
        );
    }


    double getBoxVolume(const BoundingBox& b) {
        return (b.x_max - b.x_min) * (b.y_max - b.y_min) * (b.z_max - b.z_min);
    }

    double getBallVolume(const Ball& ball) {
        return (4.0 / 3.0) * M_PI * ball.radius * ball.radius * ball.radius;
    }

    /**
     * @brief Volume of the intersection (box n ball).
     *
     * Exact when the box lies entirely inside the ball (the intersection is then
     * the box itself); estimated by Monte-Carlo sampling of the box otherwise.
     *
     * @return Volume in the same units as the box, i.e. m^3.
     */
    double getIntersectionVolume(const BoundingBox& b, const Ball& ball) {
        const double box_vol = getBoxVolume(b);
        if (box_vol <= 1e-9) return 0.0;

        // Full containment => box n ball == box, no sampling needed.
        if (isBoxFullyInBall(b, ball)) return box_vol;

        const double r_sq = ball.radius * ball.radius;

        const int samples = 1000;
        int inside_count = 0;

        // Use local distributions to sample inside this specific box
        std::uniform_real_distribution<double> x_d(b.x_min, b.x_max);
        std::uniform_real_distribution<double> y_d(b.y_min, b.y_max);
        std::uniform_real_distribution<double> z_d(b.z_min, b.z_max);

        for (int i = 0; i < samples; ++i) {
            const double dx = x_d(rng_) - ball.center.x();
            const double dy = y_d(rng_) - ball.center.y();
            const double dz = z_d(rng_) - ball.center.z();

            if (dx*dx + dy*dy + dz*dz <= r_sq) {
                inside_count++;
            }
        }

        return box_vol * (static_cast<double>(inside_count) / samples);
    }

    /**
     * @brief Fraction of the ball's volume accounted for by the box.
     *
     * This is the single normalization point for ball-vs-box overlap: both the
     * fully-contained and the partially-intersecting case go through it, so the
     * value is always comparable across nodes and across targets.
     *
     * @return Dimensionless fraction in [0, 1].
     */
    double getBallFraction(const BoundingBox& b, const Ball& ball) {
        return getIntersectionVolume(b, ball) / getBallVolume(ball);
    }

};

} // namespace visual_planner
