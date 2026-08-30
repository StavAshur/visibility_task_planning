#pragma once

#include <memory>
#include <string>

#include <ros/ros.h>

#include "VisPRMPlanner.h"
#include "VisRRTPlanner.h"
#include "VisibilityPlannerBase.h"

namespace visual_planner {

/**
 * @brief Everything a planner needs configured that is not part of the shared context.
 *
 * Grouped into one struct so a caller can describe a planner in a single value, rather
 * than reaching for a setter per option on a base pointer it would have to downcast.
 */
struct PlannerOptions {
    /// Snap a vertex being added, whose position sees a target, into one that looks at
    /// it. VisRRT's plan() and both planners' coverage queries.
    bool snap_fov_on_insert = true;
    /// The same while scanning vertices the graph already held (VisPRM coverage only).
    bool snap_fov_on_scan = false;
    bool use_visibility_integrity = true; ///< Draw goal samples from the visibility structure.
    bool shortcutting = true;
    int time_cap = 120;
    RRTParams rrt;
    PRMParams prm;
};

/**
 * @brief Builds the planner named by `mode` on the given shared context.
 *
 * Adding a planner means adding a branch here rather than touching the caller's
 * control flow. Returns nullptr for an unrecognised mode so the caller can decide
 * whether to fall back or fail.
 *
 * @param mode One of "VisRRT" or "VisPRM".
 * @param ctx  The shared world. Several planners may hold the same one, which is what
 *             lets a mode switch avoid rebuilding the VI-tree.
 */
inline std::unique_ptr<VisibilityPlannerBase> createPlanner(
        const std::string& mode,
        const std::shared_ptr<PlanningContext>& ctx,
        const PlannerOptions& opts = PlannerOptions())
{
    std::unique_ptr<VisibilityPlannerBase> planner;

    if (mode == "VisRRT") {
        planner.reset(new VisRRTPlanner(ctx, opts.snap_fov_on_insert));
    } else if (mode == "VisPRM") {
        planner.reset(new VisPRMPlanner(ctx));
    } else {
        ROS_ERROR("[PlannerFactory] Unknown planner mode '%s'.", mode.c_str());
        return nullptr;
    }

    planner->setSnapFovOnInsert(opts.snap_fov_on_insert);
    planner->setSnapFovOnScan(opts.snap_fov_on_scan);
    planner->setShortcutting(opts.shortcutting);
    planner->setTimeCap(opts.time_cap);
    planner->setRRTParams(opts.rrt);
    planner->setPRMParams(opts.prm);
    planner->setUseVisibilityIntegrity(opts.use_visibility_integrity);

    return planner;
}

} // namespace visual_planner
