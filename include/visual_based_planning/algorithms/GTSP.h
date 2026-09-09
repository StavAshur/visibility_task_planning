#pragma once

#include <vector>
#include <string>
#include <limits>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <utility>

/**
 * @file GTSP.h
 * @brief Heuristics for the Equality Generalized Travelling Salesman Problem
 *        (E-GTSP): three insertion constructions plus a 2-opt improvement pass.
 *
 * This is step 3 of claude_context/global_plan.txt, specified in
 * claude_context/GTSP_implementation_plan.txt. Read that plan alongside this
 * file: the section numbers quoted throughout (1.1, 2.4, 5.2, D6, ...) refer
 * to it, and every non-obvious choice here is argued there rather than twice.
 *
 * SCOPE -- THIS FILE IS INDEX-ONLY. It knows nothing about roadmaps, robot
 * configurations, visibility or targets. It sees a cost matrix, a cluster
 * label per node and a depot. That is deliberate: the same four algorithms are
 * wanted again in step 6, and keeping them free of planner types is what makes
 * that reuse possible. The translation from a coverage result into a
 * GTSPInstance -- and in particular the NODE SPLITTING of decision D3 -- lives
 * in planners/CoverageTour.h, not here.
 *
 * THE PROBLEM. The nodes are partitioned into clusters. A feasible tour visits
 * EXACTLY ONE node per cluster and returns to its start (D1, a cycle). One
 * cluster is the depot's, a singleton, so "the tour must start at the start
 * configuration" needs no special case anywhere below (1.3).
 *
 * WHY EXACTLY ONE PER CLUSTER MATTERS. Under D3 each cluster is one target and
 * its nodes are the configurations that see it, so a tour with one node per
 * cluster covers every target BY CONSTRUCTION. There is therefore no coverage
 * bookkeeping in this file at all: no uncovered set, no coverage bitmask, no
 * re-validation after a 2-opt move or an RP1 substitution. That saving is the
 * main reason D3 was chosen.
 *
 * THE COST MATRIX IS A PSEUDOMETRIC, NOT A METRIC (5. of the plan). It obeys
 * the triangle inequality, so every insertion cost is non-negative (1.1), but
 * splitting puts DISTINCT nodes at distance 0 -- the copies of one physical
 * configuration that sees several targets. Three consequences are load-bearing
 * and each is enforced below where it applies:
 *   - strict improvement only, gain > kGTSPEpsilon, never >= 0 (5.1);
 *   - deterministic tie-breaking, since exact ties are now systematic (5.2);
 *   - never infer node identity from a zero cost (5.3).
 */

namespace visual_planner {

/**
 * @brief Strict-improvement threshold for the 2-opt pass (plan 5.1).
 *
 * Zero-cost node pairs are systematic under D3, so gain == 0 moves are the
 * normal case rather than a measure-zero accident. Accepting them would let
 * the local search cycle between equal-cost tours forever, so every acceptance
 * test below is `gain > kGTSPEpsilon` and never `gain >= 0`.
 */
static constexpr double kGTSPEpsilon = 1e-9;

/**
 * @brief Examine 2-opt's edge pairs in a random order rather than lexicographic.
 *
 * Hardcoded rather than a parameter on purpose: it changes the character of the
 * search, not a per-call detail, so it should be switched here and recompiled.
 *
 * WHY IT IS WORTH DOING. twoOpt() is a FIRST-improvement search -- it applies
 * each improving move as it finds it -- so the local optimum it lands in
 * depends on the order the pairs are visited. Lexicographic order biases that
 * systematically toward moves among the low-index stops. Shuffling removes the
 * bias and, with a varied seed, turns the pass into a cheap multi-restart.
 * Under BEST-improvement the order would not matter at all except for ties.
 *
 * WHY IT IS SAFE. Order cannot affect correctness. Every accepted move still
 * strictly decreases the true tour cost (5.1), which is what buys termination
 * -- finitely many tours, cost strictly decreasing, so no tour repeats -- and
 * the guarantee that the output is never worse than the input. Nothing in the
 * gain accounting or in RP1's decomposition reads the scan order.
 */
static constexpr bool kGTSPRandomizePairOrder = true;

/**
 * @brief Fixed seed for the pair shuffle. Reproducibility, deliberately.
 *
 * A random scan order would otherwise reintroduce exactly what 5.2's
 * tie-breaking rules exist to prevent: the same instance yielding a different
 * tour on every run, which makes comparing the four algorithms meaningless. A
 * FIXED seed keeps the exploration benefit and keeps runs reproducible. To do
 * multiple restarts, vary this deliberately and record which value produced a
 * result -- do not seed from the clock or std::random_device.
 *
 * Note this does NOT replace 5.2: the argmins inside bestEndpointPair() and
 * the insertion steps must still break ties deterministically, or a fixed seed
 * would not reproduce anything.
 */
static constexpr unsigned kGTSPPairOrderSeed = 20260909u;

/**
 * @brief An E-GTSP instance over node indices [0, n).
 *
 * Built by planners/CoverageTour.h. The clusters must PARTITION the nodes --
 * that is decision D3, and `cluster` being a single int per node rather than a
 * list is that decision expressed in the type. It is also what makes RP1's
 * replacement-candidate set unambiguous (2.5b): "the other nodes of this
 * stop's cluster" is meaningless if a node can belong to several.
 */
struct GTSPInstance {
    /// n x n symmetric cost matrix. Infinity means "no roadmap path", which is
    /// legal input: such a node is simply never inserted (1.5).
    std::vector<std::vector<double>> cost;

    /// cluster[v] = the single cluster of node v (D3).
    std::vector<int> cluster;

    /// clusters[h] = the node indices in cluster h, ascending. Must agree with
    /// `cluster` and cover every node exactly once; checkInstance() verifies it.
    std::vector<std::vector<int>> clusters;

    /// The depot node. Its cluster is a singleton, so the depot is forced into
    /// every tour by the one-node-per-cluster rule alone (1.3).
    int depot = -1;

    /// D1: the tour is a cycle. `false` (an open path) is NOT implemented; the
    /// entry points below refuse it loudly rather than guess a semantics.
    bool closed = true;
};

/**
 * @brief A tour over a GTSPInstance.
 *
 * `stops` is CYCLIC: the edge from stops.back() back to stops[0] is part of the
 * tour and is counted in `cost`. stops[0] is always the depot.
 */
struct GTSPTour {
    /// Node indices in visiting order; stops[0] == depot; read cyclically.
    std::vector<int> stops;

    /// Total cost of the |stops| cyclic edges. Maintained incrementally by the
    /// algorithms; validateTour() re-derives it from scratch as a cross-check.
    double cost = 0.0;

    /// True iff every cluster is served exactly once, i.e. `unserved` is empty.
    bool feasible = false;

    /// Clusters that could not be reached with finite cost, ascending. Empty
    /// iff `feasible`. A non-empty list is a genuine infeasibility to report to
    /// the caller (1.6), not a partial answer to paper over.
    std::vector<int> unserved;
};

/**
 * @brief Which selection rule the shared insertion skeleton uses (2.0).
 *
 * All three share one insertion step and differ ONLY in how the next cluster is
 * chosen, exactly as in the two source documents.
 */
enum class InsertionRule {
    NEAREST,    ///< 2.1: the cluster closest to the current tour.
    FARTHEST,   ///< 2.2: the cluster farthest from it (the PDF's E-GTSP variant).
    CHEAPEST    ///< 2.3: no separate selection; one global cheapest splice.
};


// ===========================================================================
// Internal helpers. Not part of the interface described in the plan's sec. 3.2.
// ===========================================================================
namespace gtsp_detail {

/// Infinity as the cost matrix spells it.
inline double inf() { return std::numeric_limits<double>::infinity(); }

/// True for a cost that names an actually traversable connection.
inline bool isFinite(double x) {
    return !std::isnan(x) && x < inf();
}

/// Cyclic index normalisation, correct for negative inputs (D1: everything
/// here is read modulo the tour length).
inline int wrap(int idx, int n) {
    return ((idx % n) + n) % n;
}

/**
 * @brief One candidate "put node `node` into tour edge (T[pos], T[pos+1])".
 *
 * Ordered lexicographically by (cost, node, pos), which IS the tie-break rule
 * of 5.2 -- lowest node index, then lowest position index. Ties are the normal
 * case under D3, and without a total order the three constructions would
 * return different tours on different runs over the same instance, making any
 * later comparison between them meaningless.
 */
struct InsertionCandidate {
    int    node = -1;
    int    pos  = -1;
    double cost = inf();

    bool valid() const { return node >= 0 && isFinite(cost); }

    /// Strict lexicographic "is a better candidate than".
    bool betterThan(const InsertionCandidate& o) const {
        if (!o.valid()) return valid();
        if (!valid())   return false;
        if (cost != o.cost) return cost < o.cost;
        if (node != o.node) return node < o.node;
        return pos < o.pos;
    }
};

/**
 * @brief ins(v, i, j) = c[i][v] + c[v][j] - c[i][j], the splice cost of 2.0.
 *
 * Non-negative whenever it is finite, by the triangle inequality of 1.1 --
 * which the insertion proofs of 2.4 rely on. Returns infinity if either new
 * edge is unusable; the removed edge c[i][j] is a current tour edge and is
 * therefore always finite, so no inf - inf ever arises here.
 */
inline double insertionCost(const GTSPInstance& inst, int v, int i, int j) {
    const double to_v   = inst.cost[i][v];
    const double from_v = inst.cost[v][j];
    if (!isFinite(to_v) || !isFinite(from_v)) return inf();
    return to_v + from_v - inst.cost[i][j];
}

/**
 * @brief Cheapest splice of one node into any of the tour's cyclic edges.
 *
 * Candidate positions are the |T| cycle edges (T[p], T[p+1 mod |T|]) for
 * p = 0 .. |T|-1, wrap-around included (1.4). Inserting into the wrap-around
 * edge appends after the last stop, so the depot keeps index 0 either way.
 */
inline InsertionCandidate bestInsertionForNode(const GTSPInstance& inst,
                                               const std::vector<int>& stops,
                                               int v) {
    InsertionCandidate best;
    const int t = static_cast<int>(stops.size());
    for (int p = 0; p < t; ++p) {
        InsertionCandidate cand;
        cand.node = v;
        cand.pos  = p;
        cand.cost = insertionCost(inst, v, stops[p], stops[wrap(p + 1, t)]);
        if (cand.betterThan(best)) best = cand;
    }
    return best;
}

/// Cheapest splice of any node of cluster h. The insertion step of 2.1/2.2.
inline InsertionCandidate bestInsertionInCluster(const GTSPInstance& inst,
                                                 const std::vector<int>& stops,
                                                 int h) {
    InsertionCandidate best;
    for (size_t a = 0; a < inst.clusters[h].size(); ++a) {
        InsertionCandidate cand =
            bestInsertionForNode(inst, stops, inst.clusters[h][a]);
        if (cand.betterThan(best)) best = cand;
    }
    return best;
}

/**
 * @brief d(C_h, T) = min over v in C_h, i in T of c[v][i]  (2.0).
 *
 * The SELECTION measure of 2.1 and 2.2. Note it is deliberately not the
 * insertion cost: nearest insertion asks which cluster is closest TO the tour,
 * then asks separately where that cluster is cheapest to splice IN, and the
 * two questions can pick different nodes of the same cluster (2.1).
 *
 * Because every run is seeded from the depot (D2) this is always a
 * cluster-to-TOUR distance and never a cluster-to-cluster one, which is why
 * the PDF's d_hk table is not needed anywhere in this file.
 */
inline double clusterToTourDistance(const GTSPInstance& inst,
                                    const std::vector<int>& stops,
                                    int h) {
    double best = inf();
    for (size_t a = 0; a < inst.clusters[h].size(); ++a) {
        const int v = inst.clusters[h][a];
        for (size_t s = 0; s < stops.size(); ++s) {
            const double d = inst.cost[v][stops[s]];
            if (isFinite(d) && d < best) best = d;
        }
    }
    return best;
}

/**
 * @brief Structural check of an instance before any algorithm touches it.
 *
 * Verifies the D3 partition explicitly rather than trusting the builder: RP1
 * and the whole no-coverage-bookkeeping argument are only sound if the
 * clusters really do partition the nodes. Fails loudly and refuses to run.
 */
inline bool checkInstance(const GTSPInstance& inst, const char* who) {
    const int n = static_cast<int>(inst.cost.size());
    if (n == 0) {
        std::cerr << "[GTSP] Error: " << who << ": empty instance." << std::endl;
        return false;
    }
    if (static_cast<int>(inst.cluster.size()) != n) {
        std::cerr << "[GTSP] Error: " << who << ": cluster[] has "
                  << inst.cluster.size() << " entries for " << n << " nodes."
                  << std::endl;
        return false;
    }
    for (int v = 0; v < n; ++v) {
        if (static_cast<int>(inst.cost[v].size()) != n) {
            std::cerr << "[GTSP] Error: " << who << ": cost matrix row " << v
                      << " is not " << n << " wide." << std::endl;
            return false;
        }
    }
    if (inst.depot < 0 || inst.depot >= n) {
        std::cerr << "[GTSP] Error: " << who << ": depot " << inst.depot
                  << " is out of range." << std::endl;
        return false;
    }
    if (!inst.closed) {
        // D1 settled the tour as a cycle "for now"; the open-path variant was
        // never designed. Guess nothing -- say so and stop.
        std::cerr << "[GTSP] Error: " << who << ": GTSPInstance::closed == false"
                  << " (open path) is not implemented; D1 specifies a cycle."
                  << std::endl;
        return false;
    }

    // The D3 partition: every node in exactly one cluster, and `cluster` and
    // `clusters` telling the same story.
    const int num_clusters = static_cast<int>(inst.clusters.size());
    std::vector<int> seen(n, 0);
    for (int h = 0; h < num_clusters; ++h) {
        for (size_t a = 0; a < inst.clusters[h].size(); ++a) {
            const int v = inst.clusters[h][a];
            if (v < 0 || v >= n) {
                std::cerr << "[GTSP] Error: " << who << ": cluster " << h
                          << " lists out-of-range node " << v << "." << std::endl;
                return false;
            }
            if (inst.cluster[v] != h) {
                std::cerr << "[GTSP] Error: " << who << ": node " << v
                          << " is listed in cluster " << h << " but cluster["
                          << v << "] == " << inst.cluster[v]
                          << "; clusters must partition the nodes (D3)."
                          << std::endl;
                return false;
            }
            if (seen[v]++) {
                std::cerr << "[GTSP] Error: " << who << ": node " << v
                          << " appears in more than one cluster; clusters must"
                          << " partition the nodes (D3)." << std::endl;
                return false;
            }
        }
        if (inst.clusters[h].empty()) {
            // 1.6: an empty cluster cannot be served, so the instance is
            // infeasible by construction. The builder is supposed to have
            // reported this already, naming the target.
            std::cerr << "[GTSP] Error: " << who << ": cluster " << h
                      << " is empty; the instance is infeasible (1.6)."
                      << std::endl;
            return false;
        }
    }
    for (int v = 0; v < n; ++v) {
        if (!seen[v]) {
            std::cerr << "[GTSP] Error: " << who << ": node " << v
                      << " belongs to no cluster (D3)." << std::endl;
            return false;
        }
    }
    if (inst.clusters[inst.cluster[inst.depot]].size() != 1u) {
        std::cerr << "[GTSP] Error: " << who << ": the depot's cluster is not a"
                  << " singleton (1.3)." << std::endl;
        return false;
    }
    return true;
}

} // namespace gtsp_detail


// ===========================================================================
// Public interface (plan sec. 3.2)
// ===========================================================================

/**
 * @brief Total cost of a tour's cyclic edges, recomputed from the matrix.
 *
 * Returns infinity if any tour edge is unusable, which marks the tour as not
 * realisable rather than quietly under-reporting its length.
 */
inline double tourCost(const GTSPInstance& inst, const GTSPTour& tour) {
    const int t = static_cast<int>(tour.stops.size());
    if (t < 2) return 0.0;
    double total = 0.0;
    for (int p = 0; p < t; ++p) {
        const double w = inst.cost[tour.stops[p]]
                                 [tour.stops[gtsp_detail::wrap(p + 1, t)]];
        if (!gtsp_detail::isFinite(w)) return gtsp_detail::inf();
        total += w;
    }
    return total;
}

/**
 * @brief Re-derive a tour's cost and cluster coverage from scratch.
 *
 * The incremental bookkeeping in the insertion loop and in twoOpt() is the most
 * likely place for a bug in this file, and this check is O(n) against an O(n^2)
 * search, so call it on every result. It is the assertion that would catch a
 * 2-opt move whose reported gain did not match the gain it delivered -- the
 * failure mode D6's L_min exists to rule out (2.5b).
 *
 * @return true iff the tour visits every cluster exactly once, starts at the
 *         depot, uses only finite edges, and `cost` matches the recomputation.
 */
inline bool validateTour(const GTSPInstance& inst, const GTSPTour& tour) {
    using namespace gtsp_detail;

    const int n = static_cast<int>(inst.cost.size());
    if (tour.stops.empty()) {
        std::cerr << "[GTSP] Error: validateTour: empty tour." << std::endl;
        return false;
    }
    // stops[0] == depot is a POST-CONDITION OF THIS FILE, not an E-GTSP
    // requirement: a cycle has no distinguished start, and the depot is pinned
    // into the tour by its singleton cluster whatever its index (1.3, 1.4).
    // Both the insertion skeleton and D6's enumeration leave index 0 alone, so
    // checking it costs nothing and catches an off-by-one in either.
    if (tour.stops[0] != inst.depot) {
        std::cerr << "[GTSP] Error: validateTour: tour starts at "
                  << tour.stops[0] << ", not at the depot " << inst.depot
                  << "; index 0 should never have moved (1.4)." << std::endl;
        return false;
    }

    // Exactly one node per cluster -- the invariant that makes coverage
    // structural and therefore needs no separate coverage test (1.2).
    std::vector<int> per_cluster(inst.clusters.size(), 0);
    for (size_t s = 0; s < tour.stops.size(); ++s) {
        const int v = tour.stops[s];
        if (v < 0 || v >= n) {
            std::cerr << "[GTSP] Error: validateTour: stop " << s
                      << " is out-of-range node " << v << "." << std::endl;
            return false;
        }
        ++per_cluster[inst.cluster[v]];
    }
    for (size_t h = 0; h < per_cluster.size(); ++h) {
        if (per_cluster[h] != 1) {
            std::cerr << "[GTSP] Error: validateTour: cluster " << h
                      << " is visited " << per_cluster[h]
                      << " times; E-GTSP requires exactly once." << std::endl;
            return false;
        }
    }

    const double recomputed = tourCost(inst, tour);
    if (!isFinite(recomputed)) {
        std::cerr << "[GTSP] Error: validateTour: the tour uses an edge of"
                  << " infinite cost." << std::endl;
        return false;
    }
    // Scale the tolerance: the tour cost is a sum of |T| roadmap distances, so
    // a fixed absolute epsilon would be too tight on long tours.
    const double tol = 1e-6 * std::max(1.0, std::fabs(recomputed));
    if (std::fabs(recomputed - tour.cost) > tol) {
        std::cerr << "[GTSP] Error: validateTour: stored cost " << tour.cost
                  << " disagrees with the recomputed " << recomputed
                  << "; the incremental bookkeeping is wrong." << std::endl;
        return false;
    }
    if (tour.feasible != tour.unserved.empty()) {
        std::cerr << "[GTSP] Error: validateTour: feasible flag and unserved"
                  << " list disagree." << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief The shared insertion skeleton of 2.0, specialised by `rule`.
 *
 * All three constructions are this one function. They differ ONLY in the
 * selection step -- which cluster to serve next -- and share an identical
 * insertion step, which is also where the CHOICE OF NODE WITHIN THE CLUSTER is
 * made. That is exactly how both source documents present them.
 *
 *   NEAREST  (2.1) select argmin_h d(C_h, T), then splice cluster h cheapest.
 *   FARTHEST (2.2) select argmax_h d(C_h, T), then splice cluster h cheapest.
 *   CHEAPEST (2.3) no separate selection: one argmin over every unserved
 *                  cluster, node and position at once.
 *
 * SEEDING. Every run starts from the depot (D2), so the PDF's unrooted "two
 * mutually farthest clusters" seed is not used and its d_hk cluster-distance
 * table is not needed. The seed cluster is picked by the rule's own measure and
 * entered with its node closest to the depot.
 *
 * SPLIT COPIES COME OUT ADJACENT, PROVABLY. If a physical configuration sees
 * several targets, D3 has already turned it into one node per target joined at
 * cost 0. Sec. 2.4 proves all three rules then place those copies next to each
 * other: the splice cost beside an already-placed copy is exactly 0, and no
 * splice cost is ever negative (1.1). Nothing in this function knows that
 * splitting happened -- the zero-weight glue does the work.
 *
 * INFEASIBILITY IS REPORTED, NOT ROUTED AROUND. A cluster no finite splice can
 * reach is recorded in GTSPTour::unserved and the tour is marked infeasible
 * (1.5, 1.6). It is not silently skipped, and the tour returned is not passed
 * off as a solution.
 *
 * @return The constructed tour. Check `feasible` before using it.
 */
inline GTSPTour insertionTour(const GTSPInstance& inst, InsertionRule rule) {
    using namespace gtsp_detail;

    GTSPTour tour;
    if (!checkInstance(inst, "insertionTour")) return tour;

    const int num_clusters = static_cast<int>(inst.clusters.size());
    const int depot_cluster = inst.cluster[inst.depot];

    // U of 2.0, as a flag array so that iterating clusters in ascending index
    // order is automatic -- which is half of the 5.2 tie-break.
    std::vector<char> served(num_clusters, 0);
    served[depot_cluster] = 1;

    tour.stops.push_back(inst.depot);   // a cycle of one
    tour.cost = 0.0;

    int remaining = num_clusters - 1;

    // ---- Steps 1+2 of the source documents: seed from the depot (D2) -------
    if (remaining > 0) {
        int    seed_cluster = -1;
        double seed_measure = 0.0;
        for (int h = 0; h < num_clusters; ++h) {
            if (served[h]) continue;
            const double d = clusterToTourDistance(inst, tour.stops, h);
            if (!isFinite(d)) continue;
            // Strict comparison over ascending h: ties keep the lowest cluster
            // index, per 5.2.
            const bool better = (seed_cluster < 0) ||
                                (rule == InsertionRule::FARTHEST ? d > seed_measure
                                                                 : d < seed_measure);
            if (better) { seed_cluster = h; seed_measure = d; }
        }

        if (seed_cluster >= 0) {
            // v <- argmin over v in C_h of c[depot][v]; ties keep the lowest
            // node index (5.2).
            int    seed_node = -1;
            double seed_cost = inf();
            for (size_t a = 0; a < inst.clusters[seed_cluster].size(); ++a) {
                const int v = inst.clusters[seed_cluster][a];
                const double d = inst.cost[inst.depot][v];
                if (isFinite(d) && (seed_node < 0 || d < seed_cost)) {
                    seed_node = v;
                    seed_cost = d;
                }
            }
            if (seed_node >= 0) {
                tour.stops.push_back(seed_node);
                // A two-stop cycle traverses the depot-to-seed edge twice: out
                // and back. Counting it once here would understate the tour and
                // desynchronise the incremental cost from tourCost().
                tour.cost = 2.0 * seed_cost;
                served[seed_cluster] = 1;
                --remaining;
            }
        }
    }

    // ---- Steps 3-5: grow one cluster at a time ----------------------------
    while (remaining > 0) {
        InsertionCandidate chosen;

        if (rule == InsertionRule::CHEAPEST) {
            // 2.3: selection and insertion collapse into one minimisation over
            // everything. The cluster served is whichever the globally cheapest
            // splice happens to belong to.
            for (int h = 0; h < num_clusters; ++h) {
                if (served[h]) continue;
                InsertionCandidate cand = bestInsertionInCluster(inst, tour.stops, h);
                if (cand.betterThan(chosen)) chosen = cand;
            }
        } else {
            // 2.1 / 2.2: pick the cluster on d(C_h, T) first, then splice it.
            int    pick = -1;
            double measure = 0.0;
            for (int h = 0; h < num_clusters; ++h) {
                if (served[h]) continue;
                const double d = clusterToTourDistance(inst, tour.stops, h);
                if (!isFinite(d)) continue;
                const bool better = (pick < 0) ||
                                    (rule == InsertionRule::FARTHEST ? d > measure
                                                                     : d < measure);
                if (better) { pick = h; measure = d; }
            }
            if (pick >= 0) chosen = bestInsertionInCluster(inst, tour.stops, pick);
        }

        if (!chosen.valid()) {
            // Nothing left is reachable. Everything still unserved is genuinely
            // unreachable from the tour, so name it all and stop (1.5, 1.6).
            for (int h = 0; h < num_clusters; ++h) {
                if (!served[h]) tour.unserved.push_back(h);
            }
            break;
        }

        // Splice into the cycle edge (T[pos], T[pos+1]): the new stop goes at
        // index pos+1, so index 0 -- the depot -- never moves (1.4).
        tour.stops.insert(tour.stops.begin() + (chosen.pos + 1), chosen.node);
        tour.cost += chosen.cost;
        served[inst.cluster[chosen.node]] = 1;
        --remaining;
    }

    tour.feasible = tour.unserved.empty();
    if (!tour.feasible) {
        std::cerr << "[GTSP] Error: insertionTour: " << tour.unserved.size()
                  << " cluster(s) unreachable and left unserved:";
        for (size_t a = 0; a < tour.unserved.size(); ++a) {
            std::cerr << " " << tour.unserved[a];
        }
        std::cerr << ". The instance is infeasible (1.6)." << std::endl;
    }
    return tour;
}


// ===========================================================================
// 2-opt (plan 2.5). Both the fixed-node variant (2.5a) and RP1 (2.5b).
// ===========================================================================
namespace gtsp_detail {

/// One endpoint pair chosen by an RP1 minimisation, plus its three-term cost.
struct PairChoice {
    int    first  = -1;     ///< replacement drawn from the first cluster
    int    second = -1;     ///< replacement drawn from the second cluster
    double cost   = inf();  ///< c[p][first] + c[first][second] + c[second][s]
};

/**
 * @brief RP1's three-term minimisation (2.5b):
 *        argmin over u' in cl(u), w' in cl(w) of c[p][u'] + c[u'][w'] + c[w'][s].
 *
 * This is the PDF's "min{c_ia + c_ab + c_bh : a in C_alpha, b in C_beta}", with
 * p and s the stops immediately before and after the new edge. Only the
 * CLUSTERS of `u` and `w` are used -- the nodes themselves are passed just to
 * name those clusters, which is well defined only because D3 gives every node
 * exactly one.
 *
 * The current pair is always among the candidates, so the value returned is
 * never worse than leaving the nodes alone. That is why RP1's gain is always at
 * least the fixed-node gain of 2.5a on the same move.
 *
 * Ties keep the lowest first-node index, then the lowest second (5.2).
 * Candidates with any infinite term are skipped; if none is usable the result
 * has first == -1 and the caller must abandon the move.
 */
inline PairChoice bestEndpointPair(const GTSPInstance& inst,
                                   int p, int u, int w, int s) {
    PairChoice best;
    const std::vector<int>& cand_u = inst.clusters[inst.cluster[u]];
    const std::vector<int>& cand_w = inst.clusters[inst.cluster[w]];
    for (size_t x = 0; x < cand_u.size(); ++x) {
        const int up = cand_u[x];
        const double head = inst.cost[p][up];
        if (!isFinite(head)) continue;
        for (size_t y = 0; y < cand_w.size(); ++y) {
            const int wp = cand_w[y];
            const double mid  = inst.cost[up][wp];
            const double tail = inst.cost[wp][s];
            if (!isFinite(mid) || !isFinite(tail)) continue;
            const double total = head + mid + tail;
            if (best.first < 0 || total < best.cost) {
                best.first  = up;
                best.second = wp;
                best.cost   = total;
            }
        }
    }
    return best;
}

} // namespace gtsp_detail

/**
 * @brief 2-opt improvement pass over a finished tour (plan 2.5).
 *
 * Removes two tour edges and reconnects the two chains the other way, which
 * reverses the segment between them:
 *
 *     remove (T[p], T[p+1]) and (T[q], T[q+1])
 *     add    (T[p], T[q])   and (T[p+1], T[q+1])
 *     reverse T[p+1 .. q]
 *
 * Costs here are SYMMETRIC (an undirected roadmap under Dijkstra), so the
 * internal cost of the reversed segment is unchanged and the gain is exactly
 * the four terms above. With asymmetric costs the whole reversed segment would
 * have to be re-summed and none of this would hold.
 *
 * Reordering stops cannot break coverage -- one node per cluster is still one
 * node per cluster -- so nothing is re-validated after a move (1.2).
 *
 * @param generalized false = 2.5a, the classic move with the node set fixed.
 *                    true  = 2.5b, RP1: the same reversal, but the node used in
 *                    the clusters at the ends of the two NEW edges is re-chosen
 *                    at the same time. RP1's gain is never below 2.5a's on the
 *                    same move, since the current nodes are among its candidates.
 * @param max_passes  Safety bound only. Each accepted move strictly decreases
 *                    the cost, so termination does not depend on it.
 * @return true iff at least one move was accepted.
 *
 * ---------------------------------------------------------------------------
 * MOVE ENUMERATION -- DECISION D6
 * ---------------------------------------------------------------------------
 * A 2-opt move on a cycle is just a choice of TWO EDGES to remove, and for a
 * cycle there is exactly one way to reconnect the halves other than the
 * original. So the enumeration is the plain pair loop, p < q over edge indices,
 * which visits every move exactly once.
 *
 * The only restriction is that the two removed edges must be FAR ENOUGH APART.
 * Measured as the cyclic separation
 *
 *     L = q - p,   s = min(L, |T| - L)
 *
 * a move is enumerated iff s >= s_min, with s_min = 2 for 2.5a and 3 for RP1.
 *
 * WHY s >= 2 (both variants). s == 1 means the two removed edges share a stop.
 * Reversing then either reverses a single stop or reverses the whole cycle, and
 * both re-add the very edges they removed: gain is identically 0, so these are
 * no-ops. Skipping them saves work and nothing else.
 *
 * WHY s >= 3 FOR RP1, AND WHY THIS IS THE WHOLE CORRECTNESS ARGUMENT. RP1
 * prices its two new edges INDEPENDENTLY -- two small minimisations instead of
 * one |C|^4 joint one. That decomposition is valid only while the six edges the
 * two minimisations own are distinct and their four anchors are themselves
 * unmutated. The anchors sit at positions {p-1, q-1, p+2, q+2} and the mutated
 * stops at {p, p+1, q, q+1}; checking those four positions one at a time, the
 * two sets intersect exactly when
 *
 *     L is in {1, 2, |T|-2, |T|-1}
 *
 * and since L and |T|-L describe the same move, that set is precisely s <= 2.
 * So s >= 3 is not a conservative margin -- it is exactly the condition, and
 * every move it admits has a sound decomposition.
 *
 * THERE IS DELIBERATELY NO PER-MOVE GUARD AND NO FALLBACK PATH. The separation
 * test is part of the enumeration. Do not add a branch that prices a rejected
 * move with 2.5a's gain instead -- 2.5a covers those moves and runs first, and
 * a move scored by a different rule than the one that enumerated it was
 * rejected as a design (plan 2.5b, decided 2026-09-09).
 *
 *     What would go wrong at s == 2: the tour edge between old positions q and
 *     q-1 would be owned by BOTH minimisations and double-booked, each copy
 *     pairing one mutated endpoint with one stale anchor. Node splitting makes
 *     the harmful direction systematic -- a split copy can zero a stale term
 *     for free, so the argmin is drawn toward candidates whose realized edge
 *     cost is large. Reported gain large, realized gain negative, which breaks
 *     2-opt's contract that every accepted move strictly decreases the TRUE
 *     cost. That contract is what buys termination and the guarantee that the
 *     output is never worse than the input.
 *
 * CONSEQUENCE: since s <= floor(|T|/2) always, the move set is non-empty only
 * for |T| >= 2*s_min -- so |T| >= 6 when generalized. With one stop per target
 * plus the depot, RP1 therefore does nothing below 5 targets and 2.5a alone
 * improves those tours. That is expected, not an error.
 *
 * SCAN ORDER. The admissible pairs are collected into a list and, when
 * kGTSPRandomizePairOrder is set, shuffled once per pass from a FIXED seed.
 * See those two constants for why the shuffle helps and why the seed is fixed.
 * With the flag off the list is in lexicographic order and the search is
 * exactly the textbook one.
 *
 * THE DEPOT DOES NOT MOVE, AND WOULD NOT MATTER IF IT DID. With p < q the
 * reversed segment T[p+1 .. q] never wraps, so index 0 is never inside it and
 * stops[0] stays the depot throughout. Worth knowing that this is a
 * convenience, not a requirement: a cycle has no distinguished start, the depot
 * is pinned into the tour by being its own singleton cluster (1.3), and no cost
 * or move in this file reads its position. The only place a start genuinely
 * exists is expandTour() (6.1), which must emit a motion plan beginning at the
 * start configuration -- so that is where a rotation belongs if one is ever
 * needed, never inside this loop.
 */
inline bool twoOpt(const GTSPInstance& inst, GTSPTour& tour,
                   bool generalized, int max_passes = 100) {
    using namespace gtsp_detail;

    if (!checkInstance(inst, "twoOpt")) return false;
    if (!tour.feasible) {
        // Improving a tour that does not serve every cluster would dress up an
        // infeasible result as a better one. Refuse (1.6).
        std::cerr << "[GTSP] Error: twoOpt: refusing to improve an infeasible"
                  << " tour (" << tour.unserved.size()
                  << " cluster(s) unserved)." << std::endl;
        return false;
    }

    const int n = static_cast<int>(tour.stops.size());
    const int s_min = generalized ? 3 : 2;
    if (n < 2 * s_min) return false;   // empty move set: legitimate, not an error

    // The admissible pairs depend only on |T| and s_min, both fixed for this
    // call -- 2-opt never changes the number of stops -- so build the list once
    // outside the pass loop. |T| = m + 1, so this is a few hundred entries at
    // the problem sizes of sec. 4 and its cost is noise next to the closure's
    // Dijkstras. Positions stay valid as the tour is mutated because these are
    // POSITION pairs, not node pairs.
    std::vector<std::pair<int, int> > pairs;
    pairs.reserve(static_cast<size_t>(n) * static_cast<size_t>(n) / 2u);
    for (int p = 0; p + 1 < n; ++p) {
        for (int q = p + 1; q < n; ++q) {
            const int L = q - p;
            if (std::min(L, n - L) < s_min) continue;   // D6, see above
            pairs.push_back(std::make_pair(p, q));
        }
    }

    // Constructed once, so successive passes get DIFFERENT permutations while
    // the whole call stays reproducible from kGTSPPairOrderSeed.
    std::mt19937 rng(kGTSPPairOrderSeed);

    bool improved_any = false;
    int  pass = 0;
    for (; pass < max_passes; ++pass) {
        bool improved_pass = false;

        if (kGTSPRandomizePairOrder) {
            std::shuffle(pairs.begin(), pairs.end(), rng);
        }

        for (size_t idx = 0; idx < pairs.size(); ++idx) {
            const int p = pairs[idx].first;
            const int q = pairs[idx].second;

            const int a = p;                    // position p
            const int b = p + 1;                // position p+1  (no wrap)
            const int c = q;                    // position q
            const int d = wrap(q + 1, n);       // position q+1  (may wrap)

            const int Ta = tour.stops[a], Tb = tour.stops[b];
            const int Tc = tour.stops[c], Td = tour.stops[d];

            double     gain = 0.0;
            PairChoice m1, m2;

            if (!generalized) {
                // 2.5a: the four-term gain, node set fixed.
                const double new1 = inst.cost[Ta][Tc];
                const double new2 = inst.cost[Tb][Td];
                if (!isFinite(new1) || !isFinite(new2)) continue;
                gain = inst.cost[Ta][Tb] + inst.cost[Tc][Td] - new1 - new2;
            } else {
                // 2.5b (RP1). The four anchors, unmutated because s >= 3.
                // Only the reversed SEGMENT is wrap-free; an anchor index
                // can still fall off either end, so read them wrapped.
                const int Tam1 = tour.stops[wrap(a - 1, n)];  // T[p-1]
                const int Tbp1 = tour.stops[wrap(b + 1, n)];  // T[p+2]
                const int Tcm1 = tour.stops[wrap(c - 1, n)];  // T[q-1]
                const int Tdp1 = tour.stops[wrap(d + 1, n)];  // T[q+2]

                // The six tour edges the two minimisations replace. All
                // distinct for 3 <= L <= |T|-3, i.e. exactly for s >= 3.
                const double old_six =
                      inst.cost[Tam1][Ta] + inst.cost[Ta][Tb]
                    + inst.cost[Tb][Tbp1] + inst.cost[Tcm1][Tc]
                    + inst.cost[Tc][Td]   + inst.cost[Td][Tdp1];
                if (!isFinite(old_six)) continue;

                // min1 owns new edge A = (T[p], T[q]) between anchors
                // T[p-1] and T[q-1]; min2 owns B = (T[p+1], T[q+1])
                // between T[p+2] and T[q+2].
                m1 = bestEndpointPair(inst, Tam1, Ta, Tc, Tcm1);
                m2 = bestEndpointPair(inst, Tbp1, Tb, Td, Tdp1);
                if (m1.first < 0 || m2.first < 0) continue;

                gain = old_six - (m1.cost + m2.cost);
            }

            // 5.1: strict improvement only. Zero-gain moves are systematic
            // under D3 and accepting them makes the search cycle forever.
            if (!(gain > kGTSPEpsilon)) continue;

            // --- apply -------------------------------------------------
            // After the reversal, position b holds the old T[q] and
            // position c the old T[p+1]; positions a and d are untouched.
            std::reverse(tour.stops.begin() + b, tour.stops.begin() + c + 1);
            if (generalized) {
                tour.stops[a] = m1.first;    // replaces T[p]
                tour.stops[b] = m1.second;   // replaces T[q], now beside it
                tour.stops[c] = m2.first;    // replaces T[p+1]
                tour.stops[d] = m2.second;   // replaces T[q+1]
            }
            // Every replacement comes from the cluster it replaces, so the
            // one-node-per-cluster invariant survives untouched (1.2).

            tour.cost   -= gain;
            improved_pass = true;
            improved_any  = true;
        }

        if (!improved_pass) break;
    }

    if (pass == max_passes) {
        // Not a correctness failure -- every accepted move strictly decreased
        // the cost -- but it means the search was cut off, so say so.
        std::cerr << "[GTSP] Warning: twoOpt: hit max_passes (" << max_passes
                  << ") with improvements still being found." << std::endl;
    }
    return improved_any;
}

} // namespace visual_planner
