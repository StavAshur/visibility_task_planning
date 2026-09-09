#pragma once

#include <vector>
#include <memory>
#include <random>
#include <map>
#include <string>
#include <algorithm>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "PlanningContext.h"
#include "../data_structures/Graph.h"
#include "../data_structures/NearestNeighbor.h"
#include "../common/Types.h"

namespace visual_planner {

/**
 * @brief What a multi-target coverage query produces.
 *
 * The query's job is to find, for every target, at least k configurations that see it
 * and are reachable from the start -- step 1 of the VisCfgTSP framework in
 * claude_context/VisTSP_frameworks.txt. Its product is therefore a labelled set of
 * configurations, NOT a path: the tour over them is a later stage, and it needs
 * goal-to-goal costs rather than the start-to-goal paths a path-shaped result would
 * carry. Those costs come from the planner's own graph, which survives the query.
 *
 * Keeping k configurations per target rather than the first one found is the point of
 * the coverage parameter: the tour gets to choose which viewpoint to use for each
 * target instead of being handed one.
 */
/**
 * @brief Restricts VisualIK's solver group for a scope, restoring it however the scope
 *        is left.
 *
 * VisualIK belongs to the shared PlanningContext, so a coverage query that restricted
 * it to the arm and then returned early -- a time cap, an invalid start, any of a dozen
 * paths -- would leave every later query, plan() included, silently solving arm-only
 * against a base it believed it could move.
 */
class IKGroupGuard {
public:
    /// Restricts the solver to `group`. An empty group leaves the setting alone, so a
    /// caller need not branch on whether a constraint is active.
    IKGroupGuard(VisualIK& ik, const std::string& group)
        : ik_(ik), previous_(ik.getIKGroupOverride())
    {
        if (!group.empty()) ik_.setIKGroupName(group);
    }

    ~IKGroupGuard() {
        if (previous_.empty()) ik_.clearIKGroupName();
        else                   ik_.setIKGroupName(previous_);
    }

    IKGroupGuard(const IKGroupGuard&) = delete;
    IKGroupGuard& operator=(const IKGroupGuard&) = delete;

private:
    VisualIK& ik_;
    std::string previous_;
};

struct CoverageResult {
    /// goals[t] = graph vertices whose configuration sees target t. Vertices only
    /// enter this list once their visibility has been confirmed against the oracle.
    /// A configuration that sees several targets appears once under each of them.
    std::vector<std::vector<VertexDesc>> goals;

    /// deficit[t] = how many more REACHABLE goals target t still needs. Recomputed by
    /// coverageComplete(), because a vertex in goals[t] is not necessarily connected
    /// to the root yet -- on a roadmap it may be connected later, or never.
    std::vector<int> deficit;

    /// Whether every target reached its coverage. False after a run that ran out of
    /// time, in which case the per-target lists still hold whatever was found.
    bool complete = false;
};

/**
 * @brief Common base for the visibility-based planners.
 *
 * Holds the state of one planning run -- the graph and its nearest-neighbour index,
 * the start configuration, the target, and the resulting path -- plus the primitives
 * every planner is written in terms of: edge validation, extension, graph insertion,
 * visibility-guided goal sampling and path finalization.
 *
 * The world itself lives in a shared PlanningContext, so constructing a planner is
 * cheap and several planners can run against one already-built VI-tree. Derived
 * classes implement plan().
 *
 * Both RRTParams and PRMParams live here rather than on the individual planners
 * because the primitives below mix the two strategies: connectToGraph() connects to
 * k nearest neighbours like a PRM, extend() steps like an RRT, and either planner may
 * use either.
 */
class VisibilityPlannerBase {
protected:
    std::shared_ptr<PlanningContext> ctx_;

    // --- Per-run configuration ---
    bool shortcutting_;
    bool use_visibility_integrity_;   ///< Use visibility-guided goal sampling.
    int time_cap_;
    RRTParams rrt_params_;
    PRMParams prm_params_;

    /// Whether a vertex being ADDED, whose position sees a target while the tool points
    /// elsewhere, may be turned by VisualIK into a configuration that looks at it.
    /// Governs VisRRT's plan() and the coverage query's test of each new vertex -- the
    /// same operation at the same moment. With it off, only configurations that already
    /// look at a target count, and VisRRT degrades to an RRT succeeding on direct
    /// visibility alone.
    bool snap_fov_on_insert_;

    /// The same, while scanning vertices the graph ALREADY held -- VisPRM, at the start
    /// of a coverage query. Off by default: a persistent roadmap holds thousands of
    /// vertices placed to answer other queries, making them both the least likely to be
    /// well aimed and the most expensive population to find that out on.
    bool snap_fov_on_scan_;

    // --- Per-run state ---
    Ball target_mes_;
    std::vector<double> start_joint_values_;
    std::vector<std::vector<double>> result_path_;
    GraphManager graph_;
    NearestNeighbor nn_;
    VertexDesc root_id_;
    std::map<VertexDesc,bool> checked_vertices_;
    std::mt19937 rng_;

    // --- Per-run state of a multi-target coverage query ---
    std::vector<Ball> targets_;
    int coverage_k_;
    /// Minimum configuration-space distance between two goals credited to the SAME
    /// target. Zero disables the test. k near-identical viewpoints satisfy the coverage
    /// count while giving the tour no real choice, which is the failure this guards.
    double min_goal_separation_;
    BaseLocality locality_;
    CoverageResult coverage_;

public:
    explicit VisibilityPlannerBase(std::shared_ptr<PlanningContext> ctx)
        : ctx_(std::move(ctx)),
          shortcutting_(true),
          use_visibility_integrity_(true),
          time_cap_(120),
          snap_fov_on_insert_(true),
          snap_fov_on_scan_(false),
          root_id_(-1),
          checked_vertices_(),
          rng_(std::random_device{}()),
          coverage_k_(1),
          min_goal_separation_(0.0)
    {
        // Default the start to wherever the robot currently is; callers normally
        // override this with setStartJoints() before planning.
        start_joint_values_ = ctx_->getCurrentJoints();
    }

    virtual ~VisibilityPlannerBase() = default;

    /// Runs the planner. Result is retrieved with getResultPath().
    virtual bool plan() = 0;

    /**
     * @brief Runs the multi-target coverage query. Result is read with getCoverage().
     *
     * Answers "is every target seen by at least k configurations reachable from the
     * start", and leaves behind the configurations that answer it plus the graph they
     * live in. Deliberately separate from plan(): plan() answers a single-target
     * question with a path, this one answers a set-covering question with a labelled
     * vertex set, and the two are not the same result narrowed differently.
     *
     * @return true when every target reached its coverage. False also comes back with
     *         a usable partial result -- see CoverageResult::deficit.
     */
    virtual bool planCoverage() = 0;

    std::shared_ptr<PlanningContext> getContext() const { return ctx_; }

    // ========================================================================
    // Configuration owned by the planner
    // ========================================================================

    void setShortcutting(bool enable) { shortcutting_ = enable; }
    /// Whether path shortcutting is enabled. Added so LocalVisTSP can honour
    /// this existing flag when smoothing a tour per leg, instead of carrying a
    /// duplicate knob of its own (there was a setter but no getter).
    bool getShortcutting() const { return shortcutting_; }
    void setRRTParams(const RRTParams& params) { rrt_params_ = params; }
    void setPRMParams(const PRMParams& params) { prm_params_ = params; }
    void setTimeCap(int time_cap) { time_cap_ = time_cap; }

    void setSnapFovOnInsert(bool use) { snap_fov_on_insert_ = use; }
    bool getSnapFovOnInsert() const { return snap_fov_on_insert_; }
    void setSnapFovOnScan(bool use) { snap_fov_on_scan_ = use; }
    bool getSnapFovOnScan() const { return snap_fov_on_scan_; }

    // --- Multi-target coverage query configuration (per run) ---

    /// The targets to cover. Each is a ball, because that is what the visibility
    /// oracle and the VI-tree answer questions about.
    void setTargets(const std::vector<Ball>& targets) { targets_ = targets; }
    const std::vector<Ball>& getTargets() const { return targets_; }

    /// How many reachable configurations must see each target. One reproduces the
    /// familiar "find a viewpoint" question; more than one leaves the later tour a
    /// choice of viewpoint per target.
    void setCoverage(int k) { coverage_k_ = k; }
    int getCoverage() const { return coverage_k_; }

    void setMinGoalSeparation(double d) { min_goal_separation_ = d; }

    /// Confines the base to `delta` metres of the start configuration's base position.
    /// The centre is resolved when the run begins, since it follows the start.
    void setBaseLocality(double delta) {
        locality_.enabled = true;
        locality_.delta = delta;
    }
    void clearBaseLocality() { locality_.enabled = false; }
    const BaseLocality& getBaseLocality() const { return locality_; }

    const CoverageResult& getCoverageResult() const { return coverage_; }

    void setStartJoints(const std::vector<double>& start) { start_joint_values_ = start; }
    const std::vector<double>& getStartJoints() const { return start_joint_values_; }

    const std::vector<std::vector<double>>& getResultPath() const { return result_path_; }

    /**
     * @brief Enables visibility-guided goal sampling, and ensures the context has a
     *        structure to sample from.
     *
     * The two are driven together so that turning guidance on cannot leave the
     * planner querying a structure that was never built.
     */
    void setUseVisibilityIntegrity(bool enable) {
        use_visibility_integrity_ = enable;
        ctx_->setUseVisibilityStructure(enable);
    }

    // ========================================================================
    // Configuration forwarded to the shared context
    // ========================================================================

    /**
     * @brief Gives the nearest-neighbour index a scene, without touching the context.
     *
     * Required before planning: NearestNeighbor builds its distance metric from the
     * scene's joint model group, and returns no neighbours at all while it has none.
     * Kept separate from setPlanningScene() so a planner constructed against an
     * already-built context can be made ready without rebuilding the VI-tree.
     */
    void attachScene(const planning_scene::PlanningScenePtr& scene) {
        nn_.setPlanningScene(scene, ctx_->group_name_);
    }

    /// Rebuilds the shared context for a new scene, then attaches this planner to it.
    void setPlanningScene(const planning_scene::PlanningScenePtr& scene) {
        ctx_->setPlanningScene(scene);
        attachScene(scene);
    }

    void setWorkspaceBounds(const BoundingBox& b)               { ctx_->setWorkspaceBounds(b); }
    void setVisibilityToolParams(const VisibilityToolParams& p) { ctx_->setVisibilityToolParams(p); }
    void setVisibilityThreshold(double t)                       { ctx_->setVisibilityThreshold(t); }
    void setVisibilityIntegrityParams(const VisibilityIntegrityParams& p) { ctx_->setVisibilityIntegrityParams(p); }
    void setUseVisibilityRoadmap(bool enable)                   { ctx_->setUseVisibilityRoadmap(enable); }
    void setResolution(double res)                              { ctx_->setResolution(res); }
    void setGroupName(const std::string& group)                 { ctx_->setGroupName(group); }
    void setEELinkName(const std::string& ee_link)              { ctx_->setEELinkName(ee_link); }

    std::string getGroupName() const  { return ctx_->getGroupName(); }
    std::string getEELinkName() const { return ctx_->getEELinkName(); }
    bool isInitialized() const        { return ctx_->isInitialized(); }

    Eigen::Isometry3d solveFK(const std::vector<double>& joints) { return ctx_->solveFK(joints); }

    // --- Accessors ---
    GraphManager& getGraph()               { return graph_; }

    /**
     * @brief The graph vertex holding the start configuration (the roadmap root).
     *
     * Added for the GTSP layer (GTSP_implementation_plan.txt sec. 3.3): the
     * coverage tour's DEPOT is the start configuration, and a caller holding a
     * planner had no way to name it -- root_id_ is protected and every other
     * use of it was internal. This exposes state the caller already owns
     * indirectly, since it set the start configuration itself, and adds no
     * behaviour.
     *
     * @return The root vertex, or (VertexDesc)(-1) if no plan or coverage query
     *         has run yet and the root was never inserted. Callers must check.
     */
    VertexDesc getRoot() const             { return root_id_; }
    NearestNeighbor& getNN()               { return nn_; }
    VisualIK& getVisualIK()                { return ctx_->getVisualIK(); }
    PathSmoother& getSmoother()            { return ctx_->getSmoother(); }
    ValidityChecker& getValidityChecker()  { return ctx_->getValidityChecker(); }
    Sampler& getSampler()                  { return ctx_->getSampler(); }
    Ball getTargetMES() const              { return target_mes_; }

    // ========================================================================
    // Primitives
    // ========================================================================

    std::vector<double> interpolate(const std::vector<double>& start, const std::vector<double>& end, double t) {
        return ctx_->validity_checker_->interpolate(start, end, t);
    }

    double distance(const std::vector<double>& a, const std::vector<double>& b) {
        return ctx_->validity_checker_->distance(a, b);
    }

    bool validateEdge(const std::vector<double>& start, const std::vector<double>& end, EdgeCheckMode mode = EdgeCheckMode::BINARY_SEARCH) {
        return ctx_->validity_checker_->validateEdge(start, end, mode);
    }

    std::vector<double> extend(const std::vector<double>& start, const std::vector<double>& goal, double max_step = -1.0) {
        double step = (max_step < 0) ? rrt_params_.max_extension : max_step;
        return ctx_->validity_checker_->extend(start, goal, step);
    }

    /**
     * @brief Whether q keeps the mobile base inside the locality disk.
     *
     * Always true when no locality constraint is set, so callers may test every
     * configuration unconditionally.
     *
     * Testing configurations is enough to keep whole EDGES inside the disk: the base
     * variables interpolate linearly and the disk is convex, so a straight line between
     * two configurations that pass this test never leaves it.
     */
    bool inLocality(const std::vector<double>& q) const {
        if (!locality_.enabled) return true;
        return (ctx_->basePosition(q) - locality_.center).squaredNorm()
                   <= locality_.delta * locality_.delta;
    }

    // ========================================================================
    // 2. Planner Operations
    // ========================================================================

    void reset() {
        ROS_WARN("Clearing roadmap graph and NN tree.");
        nn_.clear();
        graph_.clear(); 
    }

    void computeTargetMES(const std::vector<geometry_msgs::Point>& targets) {
        // The arithmetic lives in enclosingBall() (common/Types.h) so that a
        // client turning a point cloud into a target ball uses the same
        // definition rather than a second copy of it. Behaviour is unchanged.
        std::vector<Eigen::Vector3d> points;
        points.reserve(targets.size());
        for (const auto& p : targets) {
            points.push_back(Eigen::Vector3d(p.x, p.y, p.z));
        }
        target_mes_ = enclosingBall(points);
        ROS_INFO("Target MES has center = (%f,%f,%f) and radius = %f",
                target_mes_.center.x(), target_mes_.center.y(), target_mes_.center.z(), target_mes_.radius);
    }

    VertexDesc addState(const std::vector<double>& q, bool compute_ee_pose = true) {
        Eigen::Isometry3d ee_pose;
        if (compute_ee_pose){
            ee_pose = solveFK(q);
        }
        else
            ee_pose = Eigen::Isometry3d::Identity();
        VertexDesc v = graph_.addVertex(q, ee_pose);
        nn_.addPoint(q, v);
        return v;
    }
    VertexDesc connectToGraph(std::vector<double>& q_connect, int k ,VertexDesc v_id = -1) {
        bool connected = false;
        VertexDesc res = v_id; // Default res to v_id
         if (v_id == root_id_){
            // ROS_WARN("VisPRM: Trying to connect start %zu vertex to graph", root_id_);
            k = k + checked_vertices_.size(); // Increase k to account for already checked vertices
	    } 

        std::vector<VertexDesc> neighbors = nn_.kNearest(q_connect, k);
        
        for (auto n_id : neighbors) {
            if (v_id == root_id_) {
                // ROS_WARN("VisPRM: Trying to connect start vertex to vertex %zu", n_id);
                if (checked_vertices_[n_id]) {
                    continue;
                }
                checked_vertices_[n_id] = true;
            }
            if (n_id == res || graph_.isEdge(n_id, res)) continue; 

            std::vector<double> q_neighbor = graph_.getVertexConfig(n_id); 

            if (validateEdge(q_connect, q_neighbor, prm_params_.edge_validation_method)) {
                // If the state hasn't been added to the graph yet, add it ONCE
                // if (v_id == root_id_)
                    // ROS_WARN("VisPRM: Connected start vertex to vertex %zu", n_id);
                if (res == -1) {
                    res = addState(q_connect);
                }
                
                // Now safely draw the edge
                graph_.addEdge(res, n_id, distance(q_connect, q_neighbor));
                connected = true;
            }
        }
        return connected ? res : -1;
    }

    /**
     * @brief A uniform configuration whose base lies inside the locality disk.
     *
     * Plain uniform sampling when no constraint is set, so callers need no branch.
     *
     * Only the base translation is replaced: the base's orientation and every arm joint
     * keep the values uniform sampling gave them, since the constraint restricts where
     * the base may stand and nothing else. The base is drawn directly rather than by
     * rejecting uniform samples, because the disk is a small fraction of the base's
     * range and rejection would discard nearly everything. Scaling the radius by
     * sqrt(u) spreads samples evenly over the disk's AREA; drawing the radius uniformly
     * would crowd them near the centre, a ring at radius r holding area proportional
     * to r.
     *
     * The draw ignores the base's joint limits, which uniform sampling would have
     * respected, so a sample outside them is rejected and redrawn -- nothing downstream
     * would catch it otherwise, ValidityChecker testing collision but never bounds.
     * That rejection costs nothing while the disk lies inside the limits, which is the
     * ordinary case.
     */
    std::vector<double> sampleLocalUniform() {
        std::vector<double> q = ctx_->sampler_->sampleUniform();
        if (!locality_.enabled) return q;

        const moveit::core::JointModelGroup* jmg =
            ctx_->planning_scene_->getRobotModel()->getJointModelGroup(ctx_->group_name_);
        std::uniform_real_distribution<double> unit(0.0, 1.0);

        for (int attempt = 0; attempt < 50; ++attempt) {
            const double radius = locality_.delta * std::sqrt(unit(rng_));
            const double angle  = 2.0 * M_PI * unit(rng_);
            q[0] = locality_.center.x() + radius * std::cos(angle);
            q[1] = locality_.center.y() + radius * std::sin(angle);

            if (jmg->satisfiesPositionBounds(q.data())) return q;
        }

        ROS_ERROR_THROTTLE(5.0, "Coverage query: the %.2f m disk around (%.2f, %.2f) barely fits "
                                "the base joint limits; sampling fell back to its centre.",
                           locality_.delta, locality_.center.x(), locality_.center.y());
        q[0] = locality_.center.x();
        q[1] = locality_.center.y();
        return q;
    }

    /**
     * @brief Samples a configuration whose end-effector sees the given target.
     *
     * Asks the context's active visibility structure for a workspace point that sees
     * the target, orients the tool toward it, and solves IK. Retries until a pose is
     * both sampled and reachable.
     *
     * The IK seed comes from sampleLocalUniform(), which matters under a locality
     * constraint: VisualIK is then restricted to the arm and cannot move the base, so
     * the seed's base position IS the solution's. Seeding from inside the disk is what
     * lets goal sampling produce goals inside it at all, and a fresh draw per attempt
     * tries the target from a spread of base placements.
     */
    bool sampleVisibilityGoal(const Ball& target, std::vector<double>& res_sample, int attempts = 100) {
        Eigen::Vector3d sample_pos;

        for (int i = 0; i < attempts; i++) {
            if (!ctx_->sampleVisibilityRegion(target, sample_pos)) continue;

            Eigen::Matrix3d sample_ori = ctx_->sampler_->computeLookAtRotation(sample_pos, target.center);

            Eigen::Isometry3d sample_pose;
            sample_pose.translation() = sample_pos;
            sample_pose.linear() = sample_ori;

            std::vector<double> ik_seed = sampleLocalUniform();

            if (ctx_->vis_ik_->solveIK(sample_pose, ik_seed, res_sample)) {
                return true;
            }
        }
        return false;
    }

    /// Samples a goal for the single-target MES. Equivalent to the overload above.
    bool sampleVisibilityGoal(std::vector<double>& res_sample, int attempts = 100) {
        return sampleVisibilityGoal(target_mes_, res_sample, attempts);
    }

    // ========================================================================
    // 3. Multi-target coverage
    // ========================================================================

    /**
     * @brief Validates and initialises the state of one coverage query.
     *
     * @return false when the query cannot meaningfully run: an empty target set or a
     *         coverage below one has no answer, and an invalid start leaves every
     *         configuration unreachable, so the planner should stop rather than search.
     */
    bool beginCoverageRun() {
        if (targets_.empty()) {
            ROS_ERROR("Coverage query: no targets set. Call setTargets() first.");
            return false;
        }
        if (coverage_k_ < 1) {
            ROS_ERROR("Coverage query: coverage is %d; it must be at least 1.", coverage_k_);
            return false;
        }
        if (start_joint_values_.empty()) {
            ROS_ERROR("Coverage query: start joint values not set.");
            return false;
        }
        if (!ctx_->validity_checker_->isValid(start_joint_values_)) {
            ROS_ERROR("Coverage query: start state is invalid.");
            return false;
        }

        coverage_.goals.assign(targets_.size(), std::vector<VertexDesc>());
        coverage_.deficit.assign(targets_.size(), coverage_k_);
        coverage_.complete = false;

        if (locality_.enabled) {
            locality_.center = ctx_->basePosition(start_joint_values_);
            ROS_INFO("Coverage query: base confined to %.2f m of (%.2f, %.2f); "
                     "VisualIK restricted to group '%s'.",
                     locality_.delta, locality_.center.x(), locality_.center.y(),
                     locality_.ik_group.c_str());
        }

        ROS_INFO("Coverage query: %lu targets, %d configuration(s) required per target.",
                 targets_.size(), coverage_k_);

        reportUnseeableTargets();
        return true;
    }

    /**
     * @brief Warns about targets that no workspace region sees well enough.
     *
     * Such a target cannot be covered however long the search runs, so without this a
     * run against one looks like a slow failure rather than an impossible request.
     * Costs one VI-tree query per coverage run.
     *
     * Advisory in one direction only. A region's visibility of a ball is an aggregate
     * over the whole region, so passing this check does not promise that any single
     * configuration inside it sees the target; failing it is strong evidence the target
     * cannot be covered. The VI-tree is also the only structure that answers this, so
     * silence while the VIR roadmap is active means nothing was checked.
     */
    void reportUnseeableTargets() {
        MultiTargetQueryResult regions;
        if (!ctx_->queryMultiTarget(targets_, regions, ctx_->visibility_threshold_)) {
            ROS_INFO("Coverage query: no VI-tree available, skipping the region check.");
            return;
        }

        for (size_t t = 0; t < targets_.size(); ++t) {
            if (regions.per_target[t].empty()) {
                ROS_WARN("Coverage query: NO workspace region sees a %.2f fraction of target %lu "
                         "(centre %.2f, %.2f, %.2f; radius %.2f). It cannot be covered.",
                         ctx_->visibility_threshold_, t,
                         targets_[t].center.x(), targets_[t].center.y(), targets_[t].center.z(),
                         targets_[t].radius);
            }
        }
    }

    /**
     * @brief Whether a vertex can be reached from the root.
     *
     * A goal that cannot be reached does not count toward coverage: the query promises
     * configurations the robot can actually get to from where it starts. Virtual so a
     * planner whose construction already guarantees reachability can say so instead of
     * rediscovering it -- see VisRRTPlanner.
     */
    virtual bool isReachable(VertexDesc v) {
        return graph_.inSameComponent(root_id_, v);
    }

    /**
     * @brief Records v as a configuration seeing target t, if it is worth recording.
     *
     * @param q The configuration at v, for the separation test.
     * @return false when the vertex was already credited to this target, or lies
     *         closer than min_goal_separation_ to one that was -- k viewpoints that are
     *         all the same viewpoint satisfy the count while leaving the later tour no
     *         choice of where to observe the target from.
     */
    bool creditTarget(size_t t, VertexDesc v, const std::vector<double>& q) {
        std::vector<VertexDesc>& goals = coverage_.goals[t];

        if (std::find(goals.begin(), goals.end(), v) != goals.end()) return false;

        if (min_goal_separation_ > 0.0) {
            for (VertexDesc g : goals) {
                if (distance(q, graph_.getVertexConfig(g)) < min_goal_separation_) return false;
            }
        }

        goals.push_back(v);
        ROS_INFO("Coverage query: target %lu now has %lu candidate configuration(s).",
                 t, goals.size());
        return true;
    }

    /**
     * @brief Recomputes every target's deficit and reports whether all are satisfied.
     *
     * Deficits are recomputed rather than carried forward because reachability changes
     * as the graph grows: a candidate that did not count last iteration may count now,
     * once the component holding it has been joined to the root's. This is the single
     * point where coverage_.deficit is refreshed, and the rest of the query reads it.
     */
    bool coverageComplete() {
        bool complete = true;

        for (size_t t = 0; t < targets_.size(); ++t) {
            int reachable = 0;
            for (VertexDesc g : coverage_.goals[t]) {
                if (isReachable(g)) reachable++;
            }

            coverage_.deficit[t] = std::max(0, coverage_k_ - reachable);
            if (coverage_.deficit[t] > 0) complete = false;
        }

        coverage_.complete = complete;
        return complete;
    }

    /**
     * @brief Tests one graph vertex against every target still short of coverage.
     *
     * A vertex serves a target in one of two ways. Either its configuration already
     * looks at the target, or its POSITION is well placed while the tool points
     * elsewhere -- in which case VisualIK is asked for a configuration at that position
     * that does look at it, and its answer joins the graph as a separate vertex hanging
     * off this one. A vertex may be credited to several targets at once, which is
     * exactly what shortens the eventual tour.
     *
     * @param pose The vertex's end-effector pose. Passed in rather than recomputed:
     *             testing n targets would otherwise repeat the same FK n times.
     * @param allow_snap Whether the VisualIK branch may run. Passed by the caller
     *             rather than read from a member because only the caller knows whether
     *             this vertex is being inserted or was already in the graph, and the
     *             two are governed separately -- see snap_fov_on_insert_/scan_.
     * @return how many targets gained a candidate configuration.
     */
    int recordVisibleTargets(VertexDesc v, const std::vector<double>& q,
                             const Eigen::Isometry3d& pose, bool allow_snap) {
        // Vertices reached by inserting are inside the disk by construction, having come
        // from sampleLocalUniform(). A scan tests whatever the graph already held, which
        // the sampler did not place there -- and a goal whose base is outside the disk
        // does not answer the question that was asked.
        if (!inLocality(q)) return 0;

        int credited = 0;

        for (size_t t = 0; t < targets_.size(); ++t) {
            if (coverage_.deficit[t] == 0) continue;
            const Ball& target = targets_[t];

            // A. The configuration already sees the target.
            if (ctx_->vis_oracle_->checkBallBeamVisibility(pose, target.center, target.radius)
                    > ctx_->visibility_threshold_) {
                if (creditTarget(t, v, q)) credited++;
                continue;
            }

            if (!allow_snap) continue;

            // B. The position is well placed but the tool points elsewhere. This test
            // builds an ideal look-at pose, so it is strictly more permissive than A --
            // no reason to run it on a vertex A already accepted.
            const Eigen::Vector3d position = pose.translation();
            if (ctx_->vis_oracle_->checkBallBeamVisibility(position, target.center, target.radius)
                    <= ctx_->visibility_threshold_) {
                continue;
            }

            Eigen::Matrix3d look_at = ctx_->sampler_->computeLookAtRotation(position, target.center);
            std::vector<double> q_snapped;
            if (!ctx_->vis_ik_->solveVisualIK(q, target, look_at, q_snapped)) continue;

            // VisualIK chooses where to put the tool from geometry alone and never
            // consults the obstacles, and the test just above answered for THIS
            // vertex's position rather than the snapped one. Crediting without asking
            // the oracle would let the result promise a configuration that sees the
            // target while it looks at a wall.
            if (ctx_->vis_oracle_->checkBallBeamVisibility(q_snapped, target.center, target.radius)
                    <= ctx_->visibility_threshold_) {
                continue;
            }

            // Restricted to the arm, VisualIK structurally cannot move the base, so this
            // holds already. It is what stands between a mistake in that restriction and
            // a silently violated constraint.
            if (!inLocality(q_snapped)) continue;

            if (!validateEdge(q, q_snapped)) continue;

            VertexDesc g = addState(q_snapped);
            graph_.addEdge(v, g, distance(q, q_snapped));

            // Connected only back to v: v must itself be reachable for this goal to
            // count, so hanging the snap off it gives the two identical reachability.
            if (creditTarget(t, g, q_snapped)) credited++;
        }

        return credited;
    }

    /// Reports the outcome per target, so a run that fell short says which targets it
    /// fell short on rather than only that it did.
    void logCoverageSummary(const char* tag, const ros::WallTime& start_time) const {
        double elapsed = (ros::WallTime::now() - start_time).toSec();

        if (coverage_.complete) {
            ROS_WARN("%s: all %lu targets covered %d time(s) in %.2f seconds.",
                     tag, targets_.size(), coverage_k_, elapsed);
            return;
        }

        ROS_WARN("%s: incomplete after %.2f seconds.", tag, elapsed);
        for (size_t t = 0; t < targets_.size(); ++t) {
            if (coverage_.deficit[t] > 0) {
                ROS_WARN("  target %lu: %lu candidate(s), still short by %d of %d.",
                         t, coverage_.goals[t].size(),
                         coverage_.deficit[t], coverage_k_);
            }
        }
    }

    /**
     * @brief A target still short of its coverage, chosen uniformly, or -1 if none are.
     *
     * Uniform over the UNSATISFIED targets, so effort rebalances by itself as targets
     * drop out of the pool.
     */
    int pickUnsatisfiedTarget() {
        std::vector<int> unsatisfied;
        unsatisfied.reserve(targets_.size());

        for (size_t t = 0; t < targets_.size(); ++t) {
            if (coverage_.deficit[t] > 0) unsatisfied.push_back(static_cast<int>(t));
        }

        if (unsatisfied.empty()) return -1;

        std::uniform_int_distribution<size_t> pick(0, unsatisfied.size() - 1);
        return unsatisfied[pick(rng_)];
    }

    // Helper to extract and smooth path
    void finalizePath(VertexDesc start, VertexDesc goal) {
        std::vector<VertexDesc> path_idx = graph_.shortestPath(start, goal);
        std::vector<std::vector<double>> raw_path;
        raw_path.reserve(path_idx.size());
        
        for (auto v : path_idx) {
            raw_path.push_back(graph_.getVertexConfig(v));
        }

        if (shortcutting_) {
            ROS_INFO("Smoothing path with shortcuts...");
            result_path_ = ctx_->path_smoother_->smoothPath(raw_path);
        } else {
            result_path_ = raw_path;
        }

        if (raw_path.size() >= 2) {
            for (size_t i = 0; i < raw_path.size() - 1; ++i) {
                if (!ctx_->validity_checker_->validateEdge(raw_path[i], raw_path[i+1])) {
                    ROS_ERROR("VisibilityPlannerBase: Final path validation FAILED at edge %lu -> %lu. The path contains collision!", i, i+1);
                    // Depending on policy, we might want to clear result_path_ or just warn.
                    // For now, we warn but proceed to smoothing if enabled (which might fix or fail).
                }
            }
        }

    }
};

} // namespace visual_planner
