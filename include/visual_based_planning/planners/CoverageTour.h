#pragma once

#include <vector>
#include <string>
#include <limits>
#include <algorithm>
#include <iostream>
#include <sstream>

#include "../data_structures/Graph.h"
#include "../algorithms/GTSP.h"
#include "VisibilityPlannerBase.h"   // for CoverageResult

/**
 * @file CoverageTour.h
 * @brief Adapter between a finished coverage query and the E-GTSP heuristics.
 *
 * Step 3 of claude_context/global_plan.txt, specified in
 * claude_context/GTSP_implementation_plan.txt sec. 3.3 and 6.1. Section numbers
 * quoted below refer to that plan.
 *
 * This file is the ONLY place that knows about NODE SPLITTING (decision D3) --
 * the trick that turns overlapping coverage sets into a textbook E-GTSP
 * instance. It sits deliberately between two layers that must not know about
 * it:
 *
 *   GraphManager (data_structures/Graph.h) plans the roadmap that VisPRM and
 *   VisRRT search. Split nodes and zero-weight glue edges must NEVER enter it:
 *   they are an artefact of the tour formulation, and a zero-weight edge
 *   between two distinct configurations would be a lie about the robot.
 *   metricClosure() therefore runs over PHYSICAL vertices only.
 *
 *   algorithms/GTSP.h is index-only, so that step 6 can reuse it. It never
 *   learns that two of its nodes are the same configuration.
 *
 * The `node_vertex` indirection below is what keeps those two facts true at
 * once, and it is also what lets a tour be expanded back into a motion plan
 * (6.1) with a table lookup instead of a search.
 *
 * WHY SPLITTING AT ALL. recordVisibleTargets() credits a vertex to EVERY target
 * it sees, so CoverageResult::goals[] has OVERLAPPING sets: a configuration
 * seeing three targets appears in three of them. E-GTSP requires the clusters
 * to partition the nodes. Splitting emits one node per (vertex, target) pair
 * and joins the copies of one vertex at cost 0, which
 *   - makes the clusters a partition, so the published algorithms apply
 *     verbatim with no overlap handling;
 *   - makes COVERAGE STRUCTURAL: one node per cluster covers every target
 *     always, so there is no coverage bookkeeping anywhere downstream;
 *   - loses nothing, because visiting the copies in succession costs 0 and
 *     plan 2.4 proves all three insertion rules actually do that.
 */

namespace visual_planner {

/**
 * @brief Knobs for building a coverage-tour instance.
 */
struct CoverageTourParams {
    /**
     * @brief D3: split a shared configuration into one node per target.
     *
     * `false` is NOT IMPLEMENTED and is refused, not approximated (sec. 8, O1).
     * The non-split formulation keeps the overlapping clusters, which means
     * cl(v) is ambiguous for a shared vertex and RP1 has no defined candidate
     * set -- running the E-GTSP code on such an instance would produce a tour
     * whose meaning nobody has worked out. Stav will design that mode later.
     */
    bool split_shared_nodes = true;
};

/**
 * @brief A GTSP instance plus everything needed to read a tour back as motion.
 *
 * Three index spaces meet here; keeping them straight is most of the work:
 *   - VertexDesc          a vertex of the roadmap graph;
 *   - closure index       a position in closure.nodes, i.e. a PHYSICAL vertex
 *                         that the tour may stop at;
 *   - gtsp node index     a position in gtsp.cost, i.e. a (vertex, target) pair
 *                         after splitting. Several gtsp nodes can share one
 *                         closure index -- that is exactly what splitting is.
 */
struct CoverageTourInstance {
    /// The index-only instance handed to algorithms/GTSP.h.
    GTSPInstance gtsp;

    /// All-pairs costs over the PHYSICAL vertices, with predecessor maps. The
    /// predecessor maps are what expandTour() walks; do not discard them.
    SubsetClosure closure;

    /// node_vertex[i] = closure index of gtsp node i's physical vertex.
    /// Two nodes with equal node_vertex are copies of ONE configuration.
    std::vector<int> node_vertex;

    /// node_target[i] = the target gtsp node i serves, or -1 for the depot.
    std::vector<int> node_target;

    /// The target a cluster serves, or -1 for the depot's cluster. Kept so
    /// failures can be reported in the caller's terms -- "target 4 has no
    /// reachable configuration" rather than "cluster 5 is empty" (1.6).
    std::vector<int> cluster_target;
};


// ===========================================================================
// Internal helpers
// ===========================================================================
namespace coverage_tour_detail {

/// Render a list of target indices for an error message.
inline std::string listTargets(const std::vector<int>& targets) {
    std::ostringstream os;
    for (size_t a = 0; a < targets.size(); ++a) {
        os << (a ? ", " : "") << targets[a];
    }
    return os.str();
}

} // namespace coverage_tour_detail


/**
 * @brief Turn a coverage result into an E-GTSP instance (plan 3.3).
 *
 * Steps, in order:
 *   1. collect the DISTINCT physical vertices {root} u goals[], and run
 *      GraphManager::metricClosure() over them -- one Dijkstra each, and the
 *      only expensive thing in the whole pipeline (sec. 4);
 *   2. drop the vertices at infinite cost from the depot. On a VisPRM roadmap a
 *      credited goal can sit in a foreign connected component, and
 *      CoverageResult::deficit counts only the reachable ones (1.5). This costs
 *      nothing extra: the depot's own row of the closure already answers it, so
 *      no inSameComponent() call and no access to the planner's internals;
 *   3. SPLIT (D3): emit one gtsp node per (surviving vertex, target crediting
 *      it) pair, plus the depot node, and fill the cost matrix from the closure
 *      by lookup -- writing 0 wherever two nodes share a physical vertex;
 *   4. refuse, naming every target left with no reachable configuration (1.6).
 *
 * SPLITTING ADDS NO DIJKSTRAS. The closure is over physical vertices; a copy's
 * row is the physical vertex's row reached through node_vertex. Splitting grows
 * the matrix, not the graph search (sec. 4).
 *
 * @param graph  The roadmap the coverage query left behind.
 * @param cov    That query's result. Only `goals` is read.
 * @param root   The start configuration's vertex -- the DEPOT. Get it from
 *               VisibilityPlannerBase::getRoot().
 * @param params See CoverageTourParams; split_shared_nodes must be true.
 * @param out    Filled on success; left cleared on failure.
 * @return false on any infeasibility or misuse, having said why. A false return
 *         is fatal for the tour: there is no partial answer worth touring
 *         (1.6).
 */
inline bool buildCoverageTourInstance(GraphManager& graph,
                                      const CoverageResult& cov,
                                      VertexDesc root,
                                      const CoverageTourParams& params,
                                      CoverageTourInstance& out) {
    out = CoverageTourInstance();

    if (!params.split_shared_nodes) {
        // O1. Not a default to fall back on -- an undesigned mode.
        std::cerr << "[CoverageTour] Error: split_shared_nodes == false is not"
                  << " implemented. Without splitting the coverage sets overlap,"
                  << " so a shared configuration has no unique cluster and RP1"
                  << " has no defined candidate set. Refusing to build an"
                  << " instance whose meaning is undefined (sec. 8, O1)."
                  << std::endl;
        return false;
    }

    const int num_targets = static_cast<int>(cov.goals.size());
    if (num_targets == 0) {
        std::cerr << "[CoverageTour] Error: the coverage result holds no"
                  << " targets; there is no tour to build." << std::endl;
        return false;
    }
    if (root >= boost::num_vertices(graph.G_)) {
        // Catches the (VertexDesc)(-1) a planner reports before it has run.
        std::cerr << "[CoverageTour] Error: root vertex " << root
                  << " is not in the graph (" << boost::num_vertices(graph.G_)
                  << " vertices). Did the coverage query run?" << std::endl;
        return false;
    }

    // --- 1. distinct physical vertices, depot first -------------------------
    // Depot first so its closure row is index 0, which step 2 then reads.
    std::vector<VertexDesc> subset;
    subset.push_back(root);
    {
        std::vector<char> collected(boost::num_vertices(graph.G_), 0);
        collected[root] = 1;
        for (int t = 0; t < num_targets; ++t) {
            for (size_t a = 0; a < cov.goals[t].size(); ++a) {
                const VertexDesc v = cov.goals[t][a];
                if (v >= boost::num_vertices(graph.G_)) {
                    std::cerr << "[CoverageTour] Error: goals[" << t
                              << "] names vertex " << v
                              << ", which is not in the graph." << std::endl;
                    return false;
                }
                if (!collected[v]) {
                    collected[v] = 1;
                    subset.push_back(v);
                }
            }
        }
    }

    out.closure = graph.metricClosure(subset);
    if (out.closure.nodes.size() != subset.size()) {
        std::cerr << "[CoverageTour] Error: metricClosure() failed."
                  << std::endl;
        out = CoverageTourInstance();
        return false;
    }

    // --- 2. drop what the depot cannot reach (1.5) -------------------------
    // closure index 0 is the depot, so its row is the reachability test. This
    // is why the closure is built before any filtering: the sweep it runs
    // anyway is the answer.
    const int closure_n = static_cast<int>(out.closure.nodes.size());
    std::vector<char> reachable(closure_n, 0);
    int num_reachable = 0;
    for (int i = 0; i < closure_n; ++i) {
        if (out.closure.cost[0][i] < std::numeric_limits<double>::infinity()) {
            reachable[i] = 1;
            ++num_reachable;
        }
    }
    if (num_reachable < closure_n) {
        std::cerr << "[CoverageTour] Note: " << (closure_n - num_reachable)
                  << " of " << closure_n << " credited configurations are"
                  << " unreachable from the start and were dropped (1.5)."
                  << std::endl;
    }

    // closure index of each vertex, for the split loop below.
    std::vector<int> closure_index_of(boost::num_vertices(graph.G_), -1);
    for (int i = 0; i < closure_n; ++i) {
        closure_index_of[out.closure.nodes[i]] = i;
    }

    // --- 4 (checked before 3): every target needs a reachable node (1.6) ---
    std::vector<int> starved;
    for (int t = 0; t < num_targets; ++t) {
        bool any = false;
        for (size_t a = 0; a < cov.goals[t].size() && !any; ++a) {
            const int ci = closure_index_of[cov.goals[t][a]];
            if (ci >= 0 && reachable[ci]) any = true;
        }
        if (!any) starved.push_back(t);
    }
    if (!starved.empty()) {
        std::cerr << "[CoverageTour] Error: no reachable configuration sees"
                  << " target(s) " << coverage_tour_detail::listTargets(starved)
                  << ", so the coverage tour is infeasible. Re-run the coverage"
                  << " query rather than touring the coverable subset (1.6)."
                  << std::endl;
        out = CoverageTourInstance();
        return false;
    }

    // --- 3. SPLIT (D3) ------------------------------------------------------
    // Cluster 0 is the depot's singleton (1.3); cluster t+1 serves target t.
    // One gtsp node per (surviving vertex, crediting target) pair.
    out.gtsp.clusters.assign(num_targets + 1, std::vector<int>());
    out.cluster_target.assign(num_targets + 1, -1);

    out.node_vertex.push_back(0);          // the depot's closure index
    out.node_target.push_back(-1);
    out.gtsp.cluster.push_back(0);
    out.gtsp.clusters[0].push_back(0);
    out.gtsp.depot = 0;

    for (int t = 0; t < num_targets; ++t) {
        const int h = t + 1;
        out.cluster_target[h] = t;
        std::vector<char> used(closure_n, 0);
        for (size_t a = 0; a < cov.goals[t].size(); ++a) {
            const int ci = closure_index_of[cov.goals[t][a]];
            if (ci < 0 || !reachable[ci]) continue;
            if (used[ci]) {
                // goals[t] should not list one vertex twice; if it does, the
                // extra copy would be a redundant node inside one cluster.
                std::cerr << "[CoverageTour] Warning: goals[" << t
                          << "] lists vertex " << out.closure.nodes[ci]
                          << " more than once; ignoring the duplicate."
                          << std::endl;
                continue;
            }
            used[ci] = 1;

            const int node = static_cast<int>(out.node_vertex.size());
            out.node_vertex.push_back(ci);
            out.node_target.push_back(t);
            out.gtsp.cluster.push_back(h);
            out.gtsp.clusters[h].push_back(node);
        }
    }

    // The start configuration may itself see targets -- planCoverage() calls
    // recordVisibleTargets() on the root before its loop -- in which case the
    // root has copies in those clusters like any other vertex, glued to the
    // depot at cost 0 by the rule below, and the tour collects them for free
    // (1.3). Nothing special is needed here; this note exists because the
    // behaviour looks surprising in a trace.

    // Cost matrix: the closure by lookup, with 0 between copies of one vertex.
    const int n = static_cast<int>(out.node_vertex.size());
    out.gtsp.cost.assign(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i == j) continue;
            out.gtsp.cost[i][j] =
                (out.node_vertex[i] == out.node_vertex[j])
                    ? 0.0                      // D3's zero-weight glue
                    : out.closure.cost[out.node_vertex[i]][out.node_vertex[j]];
        }
    }
    out.gtsp.closed = true;   // D1

    if (!gtsp_detail::checkInstance(out.gtsp, "buildCoverageTourInstance")) {
        out = CoverageTourInstance();
        return false;
    }
    return true;
}

/**
 * @brief Turn a finished tour back into a sequence of joint configurations (6.1).
 *
 * Walks the cycle and, for each consecutive pair of stops:
 *   - if the two stops SHARE a physical vertex (equal node_vertex), the leg is
 *     empty and the run of split copies collapses to one waypoint. This is
 *     where "one physical stop serves several targets" is turned back into a
 *     single stop, and it is the only place splitting is undone;
 *   - otherwise, read the roadmap path out of the closure's predecessor maps
 *     and append each vertex's joint_config.
 * The cycle is then closed back to the depot (D1) by the wrap-around pair.
 *
 * Only ADJACENT copies collapse. RP1 can leave two copies of one physical
 * vertex at non-adjacent tour positions, which expands into two real visits to
 * that configuration. That is accepted deliberately: the cost matrix prices
 * both visits correctly, so it is not a phantom gain, and merging them would
 * need a relocation (Or-opt) move that 2-opt cannot reach (plan 6.1).
 *
 * NO SMOOTHING HAPPENS HERE, ON PURPOSE. PathSmoother::smoothPath() runs a
 * Dijkstra over ALL pairs of waypoints and takes any collision-free
 * straight-line shortcut, so run on the concatenated tour it will happily cut
 * straight past the observation configurations -- the very stops the tour
 * exists to visit. Smooth ONCE PER LEG with the stops pinned as leg endpoints,
 * which is what `stop_indices` below is for.
 *
 * Every leg is already a valid roadmap path (1.1), so shortcutting is pure
 * improvement and nothing here needs an RRT fallback.
 *
 * @param tour         A feasible tour over instance.gtsp.
 * @param instance     The instance it was solved on.
 * @param graph        The roadmap, for joint_config lookups and closurePath().
 * @param stop_indices Optional. Receives, for each tour stop in order, the
 *                     index in the returned path of that stop's waypoint --
 *                     the leg boundaries. Without it a caller cannot honour
 *                     the smooth-per-leg rule above, because the concatenation
 *                     loses which waypoints were stops. Collapsed runs of
 *                     copies all report the same index, since they are one
 *                     waypoint.
 * @return The waypoints, starting at the depot. Empty on error.
 */
inline std::vector<std::vector<double> >
expandTour(const GTSPTour& tour,
           const CoverageTourInstance& instance,
           GraphManager& graph,
           std::vector<int>* stop_indices = nullptr) {
    std::vector<std::vector<double> > path;
    if (stop_indices) stop_indices->clear();

    if (!tour.feasible) {
        std::cerr << "[CoverageTour] Error: expandTour: the tour is infeasible;"
                  << " there is nothing valid to execute." << std::endl;
        return path;
    }
    const int t_len = static_cast<int>(tour.stops.size());
    if (t_len == 0) return path;

    // The first stop is the depot, and it is always a real waypoint.
    path.push_back(graph.getVertexConfig(
        instance.closure.nodes[instance.node_vertex[tour.stops[0]]]));
    if (stop_indices) stop_indices->push_back(0);

    for (int s = 0; s < t_len; ++s) {
        const int from_node = tour.stops[s];
        const int to_node   = tour.stops[(s + 1) % t_len];
        const int from_ci   = instance.node_vertex[from_node];
        const int to_ci     = instance.node_vertex[to_node];

        const bool last = (s + 1 == t_len);

        if (from_ci == to_ci) {
            // Copies of one configuration: no motion at all. Collapse -- the
            // next stop reuses the waypoint already emitted (6.1).
            if (!last && stop_indices) {
                stop_indices->push_back(static_cast<int>(path.size()) - 1);
            }
            continue;
        }

        const std::vector<VertexDesc> leg =
            graph.closurePath(instance.closure, from_ci, to_ci);
        if (leg.size() < 2) {
            std::cerr << "[CoverageTour] Error: expandTour: no roadmap path"
                      << " between closure nodes " << from_ci << " and "
                      << to_ci << ", but the tour priced this leg as finite."
                      << " The closure and the tour disagree." << std::endl;
            return std::vector<std::vector<double> >();
        }

        // leg[0] is the stop we are already standing on; append from leg[1].
        for (size_t a = 1; a < leg.size(); ++a) {
            path.push_back(graph.getVertexConfig(leg[a]));
        }
        // The wrap-around leg returns to the depot, whose waypoint is path[0];
        // it is a real move and must stay in the path, but it is not a new stop.
        if (!last && stop_indices) {
            stop_indices->push_back(static_cast<int>(path.size()) - 1);
        }
    }

    return path;
}

} // namespace visual_planner
