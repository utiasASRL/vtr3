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
 * \file privileged.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_pose_graph/evaluator_base/types.hpp"

namespace vtr {
namespace pose_graph {
namespace eval {
namespace mask {
namespace privileged {

namespace detail {

template <class GRAPH>
ReturnType computeVertex(const GRAPH &graph, const VertexId &v) {
  const auto nbs = graph.neighbors(v);
  for (const auto &nb : nbs) {
    if (graph.at(EdgeId(v, nb))->isManual()) return true;
  }
  return false;
}

template <class GRAPH>
ReturnType computeEdge(const GRAPH &graph, const EdgeId &e) {
  return graph.at(e)->isManual() ||
         (computeVertex(graph, e.id1()) && computeVertex(graph, e.id2()));
}

}  // namespace detail

template <class GRAPH>
class Eval : public BaseEval {
 public:
  PTR_TYPEDEFS(Eval);

  Eval(const GRAPH &graph) : graph_(graph) {}

 protected:
  ReturnType computeEdge(const EdgeId &e) override {
    return detail::computeEdge(graph_, e);
  }

  ReturnType computeVertex(const VertexId &v) override {
    return detail::computeVertex(graph_, v);
  }

 private:
  const GRAPH &graph_;
};

template <class GRAPH>
class CachedEval : public BaseCachedEval {
 public:
  PTR_TYPEDEFS(CachedEval);

  CachedEval(const GRAPH &graph) : graph_(graph) {}

 protected:
  ReturnType computeEdge(const EdgeId &e) override {
    return detail::computeEdge(graph_, e);
  }

  ReturnType computeVertex(const VertexId &v) override {
    return detail::computeVertex(graph_, v);
  }

 private:
  const GRAPH &graph_;
};

template <class GRAPH>
class WindowedEval : public BaseWindowedEval {
 public:
  PTR_TYPEDEFS(WindowedEval);

  WindowedEval(const GRAPH &graph, const size_t &cache_size)
      : BaseWindowedEval(cache_size), graph_(graph) {}

 protected:
  ReturnType computeEdge(const EdgeId &e) override {
    return detail::computeEdge(graph_, e);
  }

  ReturnType computeVertex(const VertexId &v) override {
    return detail::computeVertex(graph_, v);
  }

 private:
  const GRAPH &graph_;
};

}  // namespace privileged
}  // namespace mask
}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr
