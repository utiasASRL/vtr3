// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file bfs_planner.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_route_planning/bfs_planner.hpp"
#include "vtr_pose_graph/evaluator/evaluators.hpp" // Hshmat: for mapping following route ids to BFS edge blacklist

namespace vtr {
namespace route_planning {

auto BFSPlanner::path(const VertexId &from, const VertexId::List &to,
                      std::list<uint64_t> &idx) -> PathType {
  if (to.empty()) {
    std::string err{"waypoint list is empty"};
    CLOG(ERROR, "route_planning") << err;
    throw std::invalid_argument(err);
  }
  idx.clear();

  const auto priv_graph = getPrivilegedGraph();

  auto rval = path(priv_graph, from, to.front());
  idx.push_back(rval.empty() ? 0 : (rval.size() - 1));

  auto from_iter = to.begin();
  auto to_iter = std::next(from_iter);
  for (; to_iter != to.end(); ++from_iter, ++to_iter) {
    const auto segment = path(priv_graph, *from_iter, *to_iter);
    if (segment.size() > 0){
      rval.insert(rval.end(), std::next(segment.begin()), segment.end());
      idx.push_back(rval.empty() ? 0 : (rval.size() - 1));
    }
  }

  return rval;
}

auto BFSPlanner::path(const VertexId &from, const VertexId &to) -> PathType {
  return path(getPrivilegedGraph(), from, to);
}

// Hshmat: Experimental: compute path while honoring banned edges.
//
// High-level idea
// - Consumers (e.g., Navigator) can call setBannedEdges(...) to populate
//   BFSPlanner::banned_edges_ with a small set of (EdgeId) graph edges that
//   should be avoided in the next topological replan.
// - Here, we construct a boolean mask evaluator that returns false for any
//   EdgeId contained in banned_edges_. This mask is given to the Dijkstra
//   search routine so those edges are never traversed (equivalent to infinite
//   cost in practice).
// - All other edges/vertices return true and are traversable.
//
// Notes
// - We keep vertex mask always true in this initial version (we could also
//   block vertices similarly if needed in the future).
// - Weight evaluator is constant distance=1 for both edges and vertices here;
//   if you later need distance-based weights, plug an appropriate evaluator.
// - This function does not mutate the graph; it only constrains search.
auto BFSPlanner::hshmat_plan(const VertexId &from, const VertexId &to) -> PathType {
  const auto priv_graph = getPrivilegedGraph();
  // Start with a permissive mask (everything allowed), then optionally
  // replace it with a lambda-based mask that filters only banned edges.
  vtr::pose_graph::eval::mask::Ptr mask = std::make_shared<vtr::pose_graph::eval::mask::ConstEval>(true, true);
  if (!banned_edges_.empty()) {
    struct Local {
      // Build a mask that yields false for any edge in 'banned_edges'. Vertices
      // remain allowed (true). The GRAPH template parameter for the evaluator
      // is the type of the subgraph used by Dijkstra (tactic::GraphBase).
      static vtr::pose_graph::eval::mask::Ptr makeMask(
          const GraphBasePtr &graph,
          const std::unordered_set<vtr::tactic::VertexId> &banned_vertices,
          const std::unordered_set<vtr::tactic::EdgeId> &banned_edges) {
        using namespace vtr::pose_graph::eval::mask;
        // Edge predicate: return true if edge NOT in banned set; false otherwise.
        auto edgeFcn = [&, graph](const vtr::tactic::GraphBase &g, const vtr::tactic::EdgeId &e) -> ReturnType {
          (void)graph; // unused
          return banned_edges.count(e) == 0;
        };
        // Vertex predicate: currently leave all vertices enabled.
        auto vertexFcn = [&, graph](const vtr::tactic::GraphBase &g, const vtr::tactic::VertexId &v) -> ReturnType {
          (void)graph; // unused
          // could exclude vertices too if needed
          return true;
        };
        // variable::Eval lets us construct a mask from the two lambdas above.
        return std::make_shared<variable::Eval<vtr::tactic::GraphBase>>( *graph, edgeFcn, vertexFcn);
      }
    };
    mask = Local::makeMask(priv_graph, {}, banned_edges_);
  }

  // Run Dijkstra with:
  // - weights: constant (1, 1) to keep behavior equivalent to unweighted BFS;
  // - mask: the edge filter assembled above so banned edges are never expanded.
  const auto computed_path = priv_graph->dijkstraSearch(
      from, to, std::make_shared<vtr::pose_graph::eval::weight::ConstEval>(1.f, 1.f), mask);
  // Convert the traversal result to the expected PathType (list of VertexId).
  PathType rval;
  rval.reserve(computed_path->numberOfVertices());
  for (auto it = computed_path->begin(from), ite = computed_path->end();
       it != ite; ++it)
    rval.push_back(*it);
  return rval;
}

auto BFSPlanner::getGraph() const -> GraphPtr {
  if (auto graph_acquired = graph_.lock())
    return graph_acquired;
  else {
    std::string err{"Graph has expired"};
    CLOG(ERROR, "route_planning.bfs") << err;
    throw std::runtime_error(err);
  }
  return nullptr;
}

auto BFSPlanner::getPrivilegedGraph() const -> GraphBasePtr {
  // get the current privileged graph
  const auto graph = getGraph();
  using PrivEval = tactic::PrivilegedEvaluator<tactic::GraphBase>;
  auto priv_eval = std::make_shared<PrivEval>(*graph);
  return graph->getSubgraph(priv_eval);
}

auto BFSPlanner::path(const GraphBasePtr &priv_graph, const VertexId &from,
                      const VertexId &to) -> PathType {
  const auto computed_path = priv_graph->dijkstraSearch(from, to);
  PathType rval;
  rval.reserve(computed_path->numberOfVertices());
  for (auto it = computed_path->begin(from), ite = computed_path->end();
       it != ite; ++it)
    rval.push_back(*it);
  return rval;
}

}  // namespace route_planning
}  // namespace vtr