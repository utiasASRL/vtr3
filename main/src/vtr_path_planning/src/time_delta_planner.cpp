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
 * \file time_delta_planner.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_path_planning/time_delta_planner.hpp>

namespace vtr {
namespace path_planning {
#if 0
TimeDeltaPlanner::TimeDeltaPlanner(const RCGraphBase::Ptr &graph,
                                   const Config &config,
                                   const time_point &start)
    : graph_(graph), cached_from_(clock::now()), config_(config) {
  _resetEval(start);
  updatePrivileged();
}

/// \brief Get a path between two vertices
PathType TimeDeltaPlanner::path(const VertexId &from, const VertexId &to) {
  std::lock_guard<std::mutex> lck(cost_mtx_);
  _checkCache();
  return _path(from, to);
}

/// \brief Get a path to a set of vertices
PathType TimeDeltaPlanner::path(const VertexId &from, const VertexId::List &to,
                                std::list<uint64_t> *idx) {
  std::lock_guard<std::mutex> lck(cost_mtx_);
  if (to.size() == 0) {
    throw std::invalid_argument(
        "Attempted to plan a path to zero target vertices");
  }

  _checkCache();
  PathType rval = this->_path(from, to.front());

  if (idx) {
    idx->clear();
    idx->push_back(rval.empty() ? 0 : (rval.size() - 1));
  }

  auto fromIter = to.begin();
  auto toIter = std::next(fromIter);

  for (; toIter != to.end(); ++fromIter, ++toIter) {
    PathType segment = this->_path(*fromIter, *toIter);
    rval.insert(rval.end(), std::next(segment.begin()), segment.end());

    if (idx) {
      idx->push_back(rval.empty() ? 0 : (rval.size() - 1));
    }
  }

  return rval;
}

/// \brief Internal, single segment path planner.  Called by both public
/// functions.
PathType TimeDeltaPlanner::_path(const VertexId &from, const VertexId &to) {
  auto path = privileged_->dijkstraSearch(from, to, eval_);

  PathType rval;
  rval.reserve(path->numberOfVertices());

  for (auto it = path->begin(from), ite = path->end(); it != ite; ++it) {
    rval.push_back(*it);
  }

  return rval;
}

/// \brief Update the cached privileged graph
void TimeDeltaPlanner::updatePrivileged() {
  std::lock_guard<std::mutex> lck(cost_mtx_);
  privileged_ = graph_->getManualSubgraph();
}
#endif
}  // namespace path_planning
}  // namespace vtr
