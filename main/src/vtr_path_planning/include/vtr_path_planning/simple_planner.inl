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
 * \file simple_planner.inl
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_path_planning/simple_planner.hpp>

namespace vtr {
namespace path_planning {

template <class GRAPH>
PathType SimplePlanner<GRAPH>::path(const VertexId &from, const VertexId &to) {
  return _path(from, to);
}

template <class GRAPH>
PathType SimplePlanner<GRAPH>::path(const VertexId &from,
                                    const VertexId::List &to,
                                    std::list<uint64_t> *idx) {
  if (to.size() == 0) {
    throw std::invalid_argument(
        "Attempted to plan a path to zero target vertices");
  }

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

template <class GRAPH>
PathType SimplePlanner<GRAPH>::_path(const VertexId &from, const VertexId &to) {
  auto path = privileged_->dijkstraSearch(from, to);

  PathType rval;
  rval.reserve(path->numberOfVertices());

  for (auto it = path->begin(from), ite = path->end(); it != ite; ++it) {
    rval.push_back(*it);
  }

  return rval;
}

template <class GRAPH>
void SimplePlanner<GRAPH>::updatePrivileged() {
  privileged_ = graph_->getManualSubgraph();
}

}  // namespace path_planning
}  // namespace vtr
