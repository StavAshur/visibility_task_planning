#pragma once

#include <vector>
#include <string>
#include <limits>
#include <algorithm>
#include <iostream>

#include <ros/ros.h>

#include "../data_structures/Graph.h"
#include "../algorithms/GTSP.h"
#include "../components/PathSmoother.h"
#include "VisibilityPlannerBase.h"
#include "CoverageTour.h"

/**
 * @file LocalVisTSP.h
 * @brief Step 4 of claude_context/global_plan.txt: drive a coverage query and
 *        the E-GTSP heuristics end to end, from targets to a motion plan.
 *
 * NO NEW ALGORITHM LIVES HERE. This is the composition of step 2 (the planners'
 * multi-target planCoverage()) and step 3 (algorithms/GTSP.h through
 * planners/CoverageTour.h). The sketch in claude_context/local_VisTSP.txt --
 * "1. G <- LocalVisRRG(...)  2. GreekTSP(G)" -- is superseded in both halves:
 * VisRRG was dropped on 2026-08-26 in favour of coverage k plus a base-locality
 * disk, and the Greek TSP (MST, post-order, shortcut) was replaced by the GTSP
 * heuristics. The COMPOSITION, though, is exactly what that note described.
 *
 * ---------------------------------------------------------------------------
 * "LOCAL" IS A KNOB, NOT A PRECONDITION -- READ THIS BEFORE CHANGING ANYTHING
 * ---------------------------------------------------------------------------
 * Nothing in this file requires the base to be confined. What makes a run local
 * is VisibilityPlannerBase::setBaseLocality(delta), which holds the base inside
 * a disk around the start and restricts IK to the arm; leaving it unset (the
 * default) runs the very same pipeline over the whole environment.
 *
 * BOTH USES ARE INTENDED (Stav, 2026-09-09):
 *   - LOCAL, as the per-location unit that GLVisTSP's steps 5 and 6 will invoke
 *     once per base location (global_plan steps 5 and 6);
 *   - GLOBAL, as the BASELINE for the whole environment. This is the
 *     "VisCfgTSP" benchmark of claude_context/VisTSP_frameworks.txt, and its
 *     purpose is to measure what the extra layers of the elaborate algorithm
 *     actually buy. So a global run is a first-class use of this class, not a
 *     degenerate one, and no code here may assume locality_.enabled.
 *
 * WHERE THE CONFIGURATION LIVES. Targets, coverage k, the start joints and the
 * base-locality disk are all set ON THE PLANNER, where their setters already
 * exist -- duplicating them here would create a second source of truth and let
 * the two disagree. This class owns only the TOUR-side choices: which
 * heuristics to run and whether to improve them with 2-opt.
 */

namespace visual_planner {

/**
 * @brief One insertion rule's outcome, kept so the rules can be compared.
 *
 * Both costs are recorded because "what did 2-opt gain?" is a question the
 * comparison has to answer, and it is unanswerable if only the final cost
 * survives.
 */
struct LocalVisTSPRun {
    InsertionRule rule = InsertionRule::NEAREST;
    const char*   name = "";              ///< For logs and result tables.

    bool   built = false;                 ///< The construction produced a feasible tour.
    double construction_cost = std::numeric_limits<double>::infinity();
    double improved_cost     = std::numeric_limits<double>::infinity();

    bool two_opt_ran      = false;        ///< 2-opt was enabled and ran on this tour.
    bool two_opt_improved = false;        ///< ...and it accepted at least one move.

    GTSPTour tour;                        ///< The final tour for this rule.
};

/**
 * @brief Everything one LocalVisTSP::solve() produced.
 */
struct LocalVisTSPResult {
    bool solved = false;

    /// One entry per ENABLED rule, in InsertionRule order so a run is
    /// reproducible and two runs are comparable line by line.
    std::vector<LocalVisTSPRun> runs;

    /// Index into `runs` of the cheapest feasible tour, or -1.
    int best = -1;

    /// The best tour expanded into joint configurations, starting at the start
    /// configuration and returning to it (D1, a cycle). Same shape as
    /// VisibilityPlannerBase::getResultPath(), so the ROS node can publish it
    /// through the conversion it already has.
    std::vector<std::vector<double> > path;

    /// Index in `path` of each tour stop, i.e. the leg boundaries. Survives
    /// smoothing. See CoverageTour.h's expandTour() for why this is needed.
    std::vector<int> stop_indices;

    /// The instance the tours were solved on. Kept because node_target[] and
    /// cluster_target[] are what turn a tour back into "which target does this
    /// stop serve", which any inspection or visualisation will want.
    CoverageTourInstance instance;

    /// True if planCoverage() did not reach coverage k for every target. The
    /// tour may still be perfectly good -- see the note in solve().
    bool coverage_incomplete = false;

    // --- Wall-clock breakdown, in seconds -------------------------------
    // Reported per phase because a single total would be almost entirely the
    // coverage query and would hide everything the tour layer does. Plan
    // sec. 4 predicts the closure's Dijkstras dominate and that the
    // combinatorial part is negligible; these fields are what test that
    // rather than assuming it.

    /// planCoverage() -- step 2's multi-target query.
    double coverage_seconds = 0.0;
    /// buildCoverageTourInstance(), which is where metricClosure() runs its
    /// one Dijkstra per distinct physical goal vertex. Expected to dominate
    /// everything below it.
    double instance_seconds = 0.0;
    /// Every enabled insertion rule plus 2-opt. Expected to be microseconds:
    /// the rules share one cost matrix and |T| is m + 1.
    double gtsp_seconds = 0.0;
    /// expandTour() plus per-leg smoothing. Smoothing does collision checks,
    /// so this is not necessarily small.
    double expand_seconds = 0.0;
    /// The sum of the phases above, measured independently as a cross-check.
    double total_seconds = 0.0;
};

/**
 * @brief Runs a coverage query and tours its result (global_plan step 4).
 *
 * Usage: configure the PLANNER (targets, coverage, start joints, and
 * setBaseLocality() only if a local run is wanted), configure the flags here,
 * then call solve().
 */
class LocalVisTSP {
public:
    /**
     * @param planner An already-configured planner. Held by reference and not
     *        owned: PlanningContext is what lets several planners share one
     *        built VI-tree, and copying a planner would defeat that.
     */
    explicit LocalVisTSP(VisibilityPlannerBase& planner) : planner_(planner) {}

    // --- Which algorithms run (Stav, 2026-09-09) --------------------------
    // One flag per insertion rule plus one for 2-opt. When the 2-opt flag is
    // set it runs on EVERY enabled rule's tour, not just the best one, because
    // "nearest + 2-opt versus farthest + 2-opt" is the comparison that is
    // wanted and it cannot be recovered from a single improved tour.
    // All four default to true: the rules share one metric closure, so running
    // all of them costs barely more than running one (plan sec. 4).

    void setUseNearestInsertion(bool on)  { use_nearest_  = on; }
    void setUseFarthestInsertion(bool on) { use_farthest_ = on; }
    void setUseCheapestInsertion(bool on) { use_cheapest_ = on; }
    void setUseTwoOpt(bool on)            { use_two_opt_  = on; }

    bool getUseNearestInsertion() const  { return use_nearest_; }
    bool getUseFarthestInsertion() const { return use_farthest_; }
    bool getUseCheapestInsertion() const { return use_cheapest_; }
    bool getUseTwoOpt() const            { return use_two_opt_; }

    /// Splitting is D3 and its OFF mode is unimplemented; exposed so a caller
    /// can see what it is getting, and so the refusal is reachable in a test.
    void setTourParams(const CoverageTourParams& p) { tour_params_ = p; }
    const CoverageTourParams& getTourParams() const { return tour_params_; }

    const LocalVisTSPResult& getResult() const { return result_; }

    /**
     * @brief Run the whole pipeline. Result is read with getResult().
     *
     * Steps:
     *   1. planCoverage() on the planner -- step 2's multi-target query;
     *   2. buildCoverageTourInstance() -- node splitting, D3;
     *   3. every enabled insertion rule, each optionally improved by 2-opt;
     *   4. keep the cheapest, expand it to joint configurations;
     *   5. smooth PER LEG with the stops pinned, if the planner has
     *      shortcutting enabled.
     *
     * PARTIAL COVERAGE IS NOT A FAILURE HERE. planCoverage() returns false when
     * some target did not reach k confirmed configurations, but a tour needs
     * only ONE reachable configuration per target, so a deficit of k-1 is
     * still perfectly tourable. The genuine failure -- a target with NO
     * reachable configuration -- is detected and named by
     * buildCoverageTourInstance() (plan 1.6). So this records the shortfall in
     * `coverage_incomplete` and carries on rather than refusing a good tour.
     *
     * @return true if a feasible tour and a path came out.
     */
    bool solve() {
        result_ = LocalVisTSPResult();
        const ros::WallTime t_begin = ros::WallTime::now();

        // --- 0. the flags must ask for something ---------------------------
        if (!use_nearest_ && !use_farthest_ && !use_cheapest_) {
            std::cerr << "[LocalVisTSP] Error: every insertion rule is disabled,"
                      << " so there is no way to build a tour. Enable at least"
                      << " one of nearest / farthest / cheapest." << std::endl;
            result_.total_seconds = (ros::WallTime::now() - t_begin).toSec();
            return false;
        }
        if (planner_.getTargets().empty()) {
            std::cerr << "[LocalVisTSP] Error: no targets set on the planner."
                      << std::endl;
            result_.total_seconds = (ros::WallTime::now() - t_begin).toSec();
            return false;
        }

        // --- 1. step 2: the multi-target coverage query --------------------
        const ros::WallTime t_coverage = ros::WallTime::now();
        const bool complete = planner_.planCoverage();
        result_.coverage_seconds = (ros::WallTime::now() - t_coverage).toSec();
        result_.coverage_incomplete = !complete;
        if (!complete) {
            std::cerr << "[LocalVisTSP] Note: planCoverage() did not reach the"
                      << " requested coverage for every target. Continuing: a"
                      << " tour needs only one reachable configuration per"
                      << " target, and a target with none is refused below."
                      << std::endl;
        }

        // --- 2. step 3's adapter: split into an E-GTSP instance ------------
        const ros::WallTime t_instance = ros::WallTime::now();
        const bool built = buildCoverageTourInstance(planner_.getGraph(),
                                                     planner_.getCoverageResult(),
                                                     planner_.getRoot(),
                                                     tour_params_,
                                                     result_.instance);
        result_.instance_seconds = (ros::WallTime::now() - t_instance).toSec();
        if (!built) {
            std::cerr << "[LocalVisTSP] Error: could not build a tour instance."
                      << std::endl;
            result_.total_seconds = (ros::WallTime::now() - t_begin).toSec();
            return false;
        }
        const GTSPInstance& inst = result_.instance.gtsp;

        // --- 3. every enabled rule, in enum order for reproducibility ------
        const ros::WallTime t_gtsp = ros::WallTime::now();
        addRun(InsertionRule::NEAREST,  "nearest",  use_nearest_);
        addRun(InsertionRule::FARTHEST, "farthest", use_farthest_);
        addRun(InsertionRule::CHEAPEST, "cheapest", use_cheapest_);

        for (size_t r = 0; r < result_.runs.size(); ++r) {
            LocalVisTSPRun& run = result_.runs[r];

            run.tour = insertionTour(inst, run.rule);
            if (!run.tour.feasible || !validateTour(inst, run.tour)) {
                // Cannot normally happen: buildCoverageTourInstance() has
                // already guaranteed every cluster holds a node reachable from
                // the depot, and all survivors share the depot's component, so
                // every pairwise cost is finite. Reaching here means the
                // instance and the algorithms disagree, which is a bug, not a
                // hard problem instance.
                std::cerr << "[LocalVisTSP] Error: " << run.name
                          << " insertion produced no valid tour on an instance"
                          << " that was already checked feasible." << std::endl;
                continue;
            }
            run.built = true;
            run.construction_cost = run.tour.cost;
            run.improved_cost     = run.tour.cost;

            if (use_two_opt_) {
                // D5: 2-opt means the generalized RP1 form. The fixed-node
                // variant (plan 2.5a) is implemented and reachable by passing
                // false, but is not wired to a flag -- say so if the ablation
                // is ever wanted.
                run.two_opt_ran = true;
                run.two_opt_improved = twoOpt(inst, run.tour, /*generalized=*/true);
                if (!validateTour(inst, run.tour)) {
                    std::cerr << "[LocalVisTSP] Error: the tour from " << run.name
                              << " insertion failed validation after 2-opt; its"
                              << " reported gains did not match the tour."
                              << std::endl;
                    run.built = false;
                    continue;
                }
                run.improved_cost = run.tour.cost;
            }
        }

        // --- 4. keep the cheapest ------------------------------------------
        // Strict <, scanning in order, so ties keep the earlier rule and the
        // choice is reproducible (the same reasoning as plan 5.2).
        for (size_t r = 0; r < result_.runs.size(); ++r) {
            if (!result_.runs[r].built) continue;
            if (result_.best < 0 ||
                result_.runs[r].improved_cost <
                    result_.runs[result_.best].improved_cost) {
                result_.best = static_cast<int>(r);
            }
        }
        // The GTSP phase ends here: construction, 2-opt and choosing a winner
        // are all pure combinatorics over the cost matrix.
        result_.gtsp_seconds = (ros::WallTime::now() - t_gtsp).toSec();

        if (result_.best < 0) {
            std::cerr << "[LocalVisTSP] Error: no enabled rule produced a"
                      << " feasible tour." << std::endl;
            result_.total_seconds = (ros::WallTime::now() - t_begin).toSec();
            return false;
        }

        // --- 5. expand, then smooth per leg --------------------------------
        const ros::WallTime t_expand = ros::WallTime::now();
        result_.path = expandTour(result_.runs[result_.best].tour,
                                  result_.instance,
                                  planner_.getGraph(),
                                  &result_.stop_indices);
        if (result_.path.empty()) {
            std::cerr << "[LocalVisTSP] Error: the best tour did not expand into"
                      << " a path." << std::endl;
            result_.expand_seconds = (ros::WallTime::now() - t_expand).toSec();
            result_.total_seconds  = (ros::WallTime::now() - t_begin).toSec();
            return false;
        }

        if (planner_.getShortcutting()) {
            smoothPerLeg();
        }
        result_.expand_seconds = (ros::WallTime::now() - t_expand).toSec();

        result_.total_seconds = (ros::WallTime::now() - t_begin).toSec();
        result_.solved = true;
        return true;
    }

private:
    void addRun(InsertionRule rule, const char* name, bool enabled) {
        if (!enabled) return;
        LocalVisTSPRun run;
        run.rule = rule;
        run.name = name;
        result_.runs.push_back(run);
    }

    /**
     * @brief Shortcut each leg separately, with the tour's stops pinned.
     *
     * WHY NOT JUST SMOOTH THE PATH. PathSmoother::smoothPath() runs a Dijkstra
     * over ALL pairs of waypoints and takes any collision-free straight line,
     * so on a whole tour it cuts straight past the observation configurations
     * -- the very stops the tour exists to visit. Handing it one leg at a time,
     * with the stops as the legs' endpoints, makes that impossible: smoothPath()
     * reconstructs from the last index back to the first, so it always keeps
     * both endpoints of what it is given.
     *
     * Shortcutting here is pure improvement, never a feasibility step: every
     * leg is already a valid roadmap walk (plan 1.1), so nothing needs an RRT
     * fallback. (The planned RRT shortcut augmentation, plan 6.2, is deferred.)
     *
     * A collapsed run of split copies has two equal consecutive stop indices,
     * which is an empty leg and is skipped -- so a configuration serving
     * several targets is pinned once and serves them all.
     */
    void smoothPerLeg() {
        const std::vector<std::vector<double> >& path = result_.path;
        const std::vector<int>& stops = result_.stop_indices;
        if (stops.empty() || path.size() < 3) return;

        std::vector<std::vector<double> > out;
        std::vector<int> out_stops;

        out.push_back(path[stops[0]]);      // the depot; stops[0] is always 0
        out_stops.push_back(0);

        const int num_stops = static_cast<int>(stops.size());
        for (int k = 0; k < num_stops; ++k) {
            const int begin = stops[k];
            // The last leg closes the cycle, ending at the final waypoint --
            // the depot again -- which is not a new stop.
            const int end = (k + 1 < num_stops) ? stops[k + 1]
                                                : static_cast<int>(path.size()) - 1;

            if (end <= begin) {
                // Collapsed copies: no motion between these two stops.
                if (k + 1 < num_stops) {
                    out_stops.push_back(static_cast<int>(out.size()) - 1);
                }
                continue;
            }

            std::vector<std::vector<double> > leg(path.begin() + begin,
                                                  path.begin() + end + 1);
            const std::vector<std::vector<double> > smoothed =
                planner_.getSmoother().smoothPath(leg);

            // smoothed[0] is the stop we are already standing on.
            for (size_t a = 1; a < smoothed.size(); ++a) {
                out.push_back(smoothed[a]);
            }
            if (k + 1 < num_stops) {
                out_stops.push_back(static_cast<int>(out.size()) - 1);
            }
        }

        result_.path.swap(out);
        result_.stop_indices.swap(out_stops);
    }

    VisibilityPlannerBase& planner_;

    bool use_nearest_  = true;
    bool use_farthest_ = true;
    bool use_cheapest_ = true;
    bool use_two_opt_  = true;

    CoverageTourParams tour_params_;
    LocalVisTSPResult  result_;
};

} // namespace visual_planner
