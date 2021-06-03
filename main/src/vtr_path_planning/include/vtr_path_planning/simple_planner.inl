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
