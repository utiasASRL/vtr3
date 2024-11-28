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
  //
  PathType rval;
  rval.reserve(computed_path->numberOfVertices());
  for (auto it = computed_path->begin(from), ite = computed_path->end();
       it != ite; ++it)
    rval.push_back(*it);
  return rval;
}

}  // namespace route_planning
}  // namespace vtr