#pragma once
#include <vector>
#include <algorithm> // for std::reverse
#include <limits>    // for infinity
#include <numeric>   // for std::iota
#include <iostream>  // for logging
#include <fstream>
#include <queue>
#include <Eigen/Dense>
#include <Eigen/Geometry> // Required for Eigen::Isometry3d

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/connected_components.hpp>

// Vertex Property
struct GraphVertex {
    std::vector<double> joint_config;
    Eigen::Isometry3d ee_pose;
};

// Edge Property
struct GraphEdge {
    double weight;
};

// Define the Boost Graph with vecS (std::vector) for storage
// This allows O(1) access by index
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, GraphVertex, GraphEdge> PlannerGraph;
typedef boost::graph_traits<PlannerGraph>::vertex_descriptor VertexDesc;
typedef boost::graph_traits<PlannerGraph>::edge_descriptor EdgeDesc;

/**
 * @brief All-pairs shortest-path costs among a SUBSET of the graph's vertices,
 *        with enough predecessor information to rebuild each pair's path.
 *
 * Produced by GraphManager::metricClosure() and consumed by the GTSP layer
 * (claude_context/GTSP_implementation_plan.txt sec. 3.1): `cost` becomes the
 * tour cost matrix, and `predecessor` is what turns a finished tour back into a
 * roadmap path without re-running any search.
 *
 * Indices are positions in `nodes`, NOT vertex descriptors. That indirection is
 * deliberate -- it is what lets the GTSP code be written against plain indices
 * and stay free of graph types.
 */
struct SubsetClosure {
    /// The subset, in index order. cost/predecessor are indexed by position here.
    std::vector<VertexDesc> nodes;

    /// n x n symmetric costs. INFINITY (not max()) means no path exists; see
    /// the note in metricClosure() for why that distinction matters.
    std::vector<std::vector<double>> cost;

    /// predecessor[i] is the full Dijkstra predecessor map rooted at nodes[i],
    /// indexed by VERTEX DESCRIPTOR over the whole graph -- so it is n x |V|.
    /// Keeping it is the entire point of this struct (see metricClosure()).
    std::vector<std::vector<VertexDesc>> predecessor;
};

class GraphManager {
public:
    PlannerGraph G_;
    
    // --- Modifiers ---

    /**
     * @brief Adds a vertex with a configuration and an end-effector pose.
     * @param q The joint configuration.
     * @param pose The Isometry3d pose associated with this vertex (e.g., end-effector pose).
     */
    VertexDesc addVertex(const std::vector<double>& q, const Eigen::Isometry3d& pose) {
        VertexDesc v = boost::add_vertex(G_);
        G_[v].joint_config = q;
        G_[v].ee_pose = pose; // Store the pose
        return v;
    }

    void addEdge(VertexDesc u, VertexDesc v, double weight) {
        GraphEdge e;
        e.weight = weight;
        boost::add_edge(u, v, e, G_);
        boost::add_edge(v, u, e, G_);
    }

    // --- Getters ---

    // Get the Joint Configuration of a specific node
    std::vector<double> getVertexConfig(VertexDesc v) const {
        return G_[v].joint_config;
    }

    // Get the End-Effector Pose of a specific node
    Eigen::Isometry3d getVertexPose(VertexDesc v) const {
        return G_[v].ee_pose;
    }

    // Get the number of vertices
    size_t getNumVertices() const {
        return boost::num_vertices(G_);
    }

    // Get number of edges
    size_t getNumEdges() const {
        return boost::num_edges(G_);
    }

    /**
     * @brief Checks if an edge exists between two vertices.
     * Uses Boost Graph's native edge lookup.
     */
    bool isEdge(VertexDesc u, VertexDesc v) {
        // boost::edge returns a pair<edge_descriptor, bool>
        std::pair<EdgeDesc, bool> result = boost::edge(u, v, G_);
        return result.second;
    }

    /**
     * @brief Finds a vertex that matches the given joint configuration.
     * @param q The joint configuration to look for.
     * @param epsilon The tolerance for floating point comparison.
     * @return A pair containing the VertexDesc and a bool (true if found, false otherwise).
     * * NOTE: This performs a linear search O(N). For fast lookups, 
     * use the NearestNeighbor structure instead.
     */
    std::pair<VertexDesc, bool> getVertex(const std::vector<double>& q, double epsilon = 1e-5) {
        // Iterate over all vertices in the graph
        auto v_range = boost::vertices(G_);
        for (auto it = v_range.first; it != v_range.second; ++it) {
            VertexDesc v = *it;
            const std::vector<double>& stored_q = G_[v].joint_config;

            if (stored_q.size() != q.size()) continue;

            bool match = true;
            for (size_t i = 0; i < q.size(); ++i) {
                if (std::abs(stored_q[i] - q[i]) > epsilon) {
                    match = false;
                    break;
                }
            }

            if (match) {
                return std::make_pair(v, true);
            }
        }

        // Return a dummy descriptor and false if not found
        // In BGL vecS, descriptors are usually integers, but we shouldn't rely on magic numbers.
        return std::make_pair(VertexDesc(), false);
    }
    // --- Algorithms ---

    /**
     * @brief Finds the shortest path between start and goal using Dijkstra's algorithm.
     * @return A vector of Vertex descriptors representing the path. Empty if no path found.
     */
    std::vector<VertexDesc> shortestPath(VertexDesc start, VertexDesc goal) {
        size_t num_nodes = boost::num_vertices(G_);
        
        // 1. Prepare vectors to store results
        std::vector<VertexDesc> predecessors(num_nodes);
        std::vector<double> distances(num_nodes);

        // 2. Run Boost Dijkstra
        // We use weight_map to tell BGL to look at our GraphEdge::weight struct member
        boost::dijkstra_shortest_paths(G_, start,
            boost::predecessor_map(&predecessors[0])
            .distance_map(&distances[0])
            .weight_map(boost::get(&GraphEdge::weight, G_))
        );

        // 3. Check if path exists
        // If distance is max (infinity), the goal is unreachable
        if (distances[goal] >= std::numeric_limits<double>::max()) {
            return {}; // Return empty vector
        }

        // 4. Reconstruct Path (Backtracking from goal to start)
        std::vector<VertexDesc> path;
        VertexDesc current = goal;

        while (current != start) {
            path.push_back(current);
            current = predecessors[current];
            
            // Safety check against cycles or disconnected components not caught by distance check
            // In a valid Dijkstra run on undirected graph, this shouldn't happen if distance < max
            if (current == predecessors[current] && current != start) {
                // This means we hit a root that isn't start (unreachable)
                return {};
            }
        }
        path.push_back(start);

        // 5. Reverse to get Start -> Goal order
        std::reverse(path.begin(), path.end());

        return path;
    }

    /**
     * @brief All-pairs shortest paths among `subset`, as a SubsetClosure.
     *
     * Runs ONE single-source Dijkstra per subset vertex and keeps both the
     * distances and the predecessor map from each.
     *
     * WHY THIS EXISTS RATHER THAN A DOUBLE LOOP OVER shortestPath().
     * shortestPath() runs a full single-source Dijkstra and then throws the
     * predecessor and distance maps away, keeping one path. Calling it for
     * every pair therefore runs n^2 Dijkstras to recover information that n
     * Dijkstras already contain, and it still cannot answer a second query from
     * the same source without starting over. Here the maps are retained, which
     * is also exactly what expanding a tour into a motion plan needs.
     *
     * UNREACHABLE PAIRS ARE INFINITY, NOT max(). BGL leaves unreached vertices
     * at std::numeric_limits<double>::max() (its default distance_inf), which
     * is a FINITE double. Downstream code tests reachability with
     * `x < infinity()`, so passing max() through would make an unreachable pair
     * look like a usable edge of astronomical cost -- a tour would be built
     * through it and reported as feasible. The conversion below is the only
     * place that is prevented, so do not remove it.
     *
     * COST: n Dijkstras, which dominates the whole GTSP pipeline -- the roadmap
     * may have PRMParams::max_size = 10000 vertices while n is a few tens. Call
     * it ONCE and reuse the result for every algorithm. Memory is O(n * |V|)
     * VertexDesc for the predecessor maps, a few MB at those sizes.
     *
     * @param subset Distinct graph vertices. Duplicates are rejected: they
     *        would silently create a zero-cost pair between two entries that
     *        are the same vertex, which is indistinguishable from -- and would
     *        corrupt -- the GTSP layer's split-copy glue.
     * @return The closure, or an empty one on error (an invalid or duplicated
     *         vertex), which callers must treat as fatal.
     */
    SubsetClosure metricClosure(const std::vector<VertexDesc>& subset) {
        SubsetClosure closure;
        const size_t num_nodes = boost::num_vertices(G_);
        const int n = static_cast<int>(subset.size());
        if (n == 0) return closure;

        // Validate before doing |subset| Dijkstras' worth of work.
        std::vector<char> already(num_nodes, 0);
        for (int i = 0; i < n; ++i) {
            if (subset[i] >= num_nodes) {
                std::cerr << "[GraphManager] Error: metricClosure: vertex "
                          << subset[i] << " is out of range (graph has "
                          << num_nodes << ")." << std::endl;
                return SubsetClosure();
            }
            if (already[subset[i]]) {
                std::cerr << "[GraphManager] Error: metricClosure: vertex "
                          << subset[i] << " appears twice in the subset; the"
                          << " subset must be distinct vertices." << std::endl;
                return SubsetClosure();
            }
            already[subset[i]] = 1;
        }

        closure.nodes = subset;
        closure.cost.assign(n, std::vector<double>(
            n, std::numeric_limits<double>::infinity()));
        closure.predecessor.resize(n);

        for (int i = 0; i < n; ++i) {
            std::vector<VertexDesc> predecessors(num_nodes);
            std::vector<double> distances(num_nodes);

            boost::dijkstra_shortest_paths(G_, subset[i],
                boost::predecessor_map(&predecessors[0])
                .distance_map(&distances[0])
                .weight_map(boost::get(&GraphEdge::weight, G_))
            );

            for (int j = 0; j < n; ++j) {
                const double d = distances[subset[j]];
                // See "UNREACHABLE PAIRS ARE INFINITY" above.
                closure.cost[i][j] = (d >= std::numeric_limits<double>::max())
                                   ? std::numeric_limits<double>::infinity()
                                   : d;
            }
            closure.predecessor[i].swap(predecessors);
        }
        return closure;
    }

    /**
     * @brief Rebuild the roadmap path between two closure entries.
     *
     * Walks the predecessor map that metricClosure() already computed for
     * `from`, so this runs no search at all -- it is a backtrack.
     *
     * @param from, to Positions in closure.nodes, not vertex descriptors.
     * @return The vertex path from closure.nodes[from] to closure.nodes[to],
     *         both endpoints included. Empty if the pair is unreachable or the
     *         indices are invalid. from == to yields the single vertex.
     */
    std::vector<VertexDesc> closurePath(const SubsetClosure& c,
                                        int from, int to) const {
        std::vector<VertexDesc> path;
        const int n = static_cast<int>(c.nodes.size());
        if (from < 0 || from >= n || to < 0 || to >= n) {
            std::cerr << "[GraphManager] Error: closurePath: index out of range"
                      << " (" << from << ", " << to << ") for " << n
                      << " closure nodes." << std::endl;
            return path;
        }
        if (!(c.cost[from][to] < std::numeric_limits<double>::infinity())) {
            return path;   // unreachable: an empty path, not an error
        }

        const std::vector<VertexDesc>& predecessors = c.predecessor[from];
        const VertexDesc start = c.nodes[from];
        VertexDesc current = c.nodes[to];

        while (current != start) {
            path.push_back(current);
            const VertexDesc parent = predecessors[current];
            // A vertex is its own predecessor only at an unreached root, so
            // this catches a predecessor map that disagrees with the distances.
            if (parent == current) {
                std::cerr << "[GraphManager] Error: closurePath: predecessor"
                          << " map is broken between closure nodes " << from
                          << " and " << to << "." << std::endl;
                return std::vector<VertexDesc>();
            }
            current = parent;
        }
        path.push_back(start);

        std::reverse(path.begin(), path.end());
        return path;
    }

    void clear() {
        G_.clear();
    }

    /**
     * @brief Computes the number of connected components and prints a representative for each.
     * @param k Optional argument. If k >= 0, prints only the k largest connected components.
     * If k < 0 (default), prints all components.
     * @return The number of disjoint subgraphs.
     */
    int countConnectedComponents(int k = -1) {
        if (boost::num_vertices(G_) == 0) return 0;
        
        // Map vertex ID -> Component ID
        std::vector<int> component(boost::num_vertices(G_));
        int num_components = boost::connected_components(G_, &component[0]);
        
        // --- MODIFIED: Track size and a representative vertex in a single pass ---
        // component_info maps Component ID -> {Size, Representative Vertex ID}
        std::vector<std::pair<int, int>> component_info(num_components, {0, -1});
        
        for (size_t v = 0; v < component.size(); ++v) {
            int c_id = component[v];
            component_info[c_id].first++; // Increment size
            
            // If we haven't assigned a representative yet, use this vertex
            if (component_info[c_id].second == -1) {
                component_info[c_id].second = v;
            }
        }

        std::cout << "[GraphManager] Total Connected Components: " << num_components << std::endl;

        // --- NEW: Sort components by descending size ---
        // Create an array of component IDs (0 to num_components - 1)
        std::vector<int> sorted_cids(num_components);
        std::iota(sorted_cids.begin(), sorted_cids.end(), 0);
        
        // Sort the IDs based on the sizes stored in component_info
        std::sort(sorted_cids.begin(), sorted_cids.end(), [&component_info](int a, int b) {
            return component_info[a].first > component_info[b].first;
        });

        // Determine how many components to actually print
        int print_limit = num_components;
        if (k >= 0) {
            print_limit = std::min(k, num_components);
            std::cout << "               (Displaying top " << print_limit << " largest)" << std::endl;
        }

        // --- MODIFIED: Print only up to the print_limit ---
        for (int i = 0; i < print_limit; ++i) {
            int c_id = sorted_cids[i];
            int size = component_info[c_id].first;
            int rep_v = component_info[c_id].second;

            std::cout << "  - Component " << c_id 
                      << " (Size: " << size << " nodes) "
                      << "Rep (Node " << rep_v << "): [";
                      
            const auto& q = G_[rep_v].joint_config;
            for (size_t j = 0; j < q.size(); ++j) {
                std::cout << q[j] << (j < q.size() - 1 ? ", " : "");
            }
            std::cout << "]" << std::endl;
        }
        
        return num_components;
    }


    /**
     * @brief Checks if two vertices are in the same connected component using BFS.
     * @param u The first vertex descriptor.
     * @param v The second vertex descriptor.
     * @return True if a path exists between u and v, false otherwise.
     */
    bool inSameComponent(VertexDesc u, VertexDesc v) {
        // Trivial case: identical vertices
        if (u == v) return true;

        size_t num_nodes = boost::num_vertices(G_);
        
        // Safety check to prevent out-of-bounds access
        if (u >= num_nodes || v >= num_nodes) return false;

        std::vector<bool> visited(num_nodes, false);
        std::queue<VertexDesc> q;

        q.push(u);
        visited[u] = true;

        while (!q.empty()) {
            VertexDesc current = q.front();
            q.pop();

            // Get the neighbors of the current vertex
            auto neighbors = boost::adjacent_vertices(current, G_);
            for (auto it = neighbors.first; it != neighbors.second; ++it) {
                VertexDesc neighbor = *it;
                
                // Early exit: We found the target vertex!
                if (neighbor == v) return true; 
                
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    q.push(neighbor);
                }
            }
        }

        // If the queue empties and we haven't found v, they are disconnected
        return false;
    }


    /**
     * @brief Computes connected components and exports all vertices to a text file.
     * @param filename The output file name (defaults to "cc_map.txt").
     */
    void exportCCMap(const std::string& filename = "cc_map.txt") {
        size_t num_nodes = boost::num_vertices(G_);
        if (num_nodes == 0) {
            std::cout << "[GraphManager] Graph is empty. Nothing to export." << std::endl;
            return;
        }

        // 1. Compute connected components for the entire graph
        std::vector<int> component(num_nodes);
        boost::connected_components(G_, &component[0]);

        // 2. Open the file for writing
        std::ofstream outfile(filename);
        if (!outfile.is_open()) {
            std::cerr << "[GraphManager] Failed to open " << filename << " for writing." << std::endl;
            return;
        }

        // 3. Find the dimension of the joint space from the first vertex
        size_t dim = 0;
        auto v_range = boost::vertices(G_);
        if (v_range.first != v_range.second) {
            dim = G_[*(v_range.first)].joint_config.size();
        }

        // 4. Write dynamic CSV header (id, j0, j1, ..., jN, cc_id)
        outfile << "id";
        for (size_t i = 0; i < dim; ++i) {
            outfile << ",j" << i;
        }
        outfile << ",cc_id\n";

        // 5. Iterate over all vertices and write their data
        for (auto it = v_range.first; it != v_range.second; ++it) {
            VertexDesc v = *it;
            const std::vector<double>& q = G_[v].joint_config;

            // Safety check to ensure the state has at least an x and y coordinate
            if (q.size() >= 2) {
                outfile << v;
                
                // Print all joint coordinates
                for (size_t i = 0; i < q.size(); ++i) {
                    outfile << "," << q[i];
                }
                
                // Append the connected component ID at the very end
                outfile << "," << component[v] << "\n";
            }
        }

        outfile.close();
        std::cout << "[GraphManager] Successfully exported " << dim << "-DOF configurations to " << filename << std::endl;
    }

    /**
     * @brief Prints the in-degree and out-degree of a specified vertex.
     * @param v_id The vertex ID to query.
     */
    void printVertexDegrees(size_t v_id) const {
        // Ensure the requested vertex actually exists in the graph
        if (v_id >= boost::num_vertices(G_)) {
            std::cerr << "[GraphManager] Error: Vertex ID " << v_id 
                      << " is out of bounds. Max ID is " 
                      << (boost::num_vertices(G_) - 1) << "." << std::endl;
            return;
        }

        // Query degrees using Boost Graph Library functions
        auto in_deg = boost::in_degree(v_id, G_);
        auto out_deg = boost::out_degree(v_id, G_);

        std::cout << "[GraphManager] Node " << v_id << " Connectivity:" << std::endl;
        std::cout << "  - In-degree:  " << in_deg << std::endl;
        std::cout << "  - Out-degree: " << out_deg << std::endl;
        std::cout << "  - Total:      " << (in_deg + out_deg) << std::endl;
    }

};
