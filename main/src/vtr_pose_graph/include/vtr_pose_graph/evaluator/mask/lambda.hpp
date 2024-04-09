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
 * \file spatial.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_pose_graph/evaluator_base/types.hpp"
#include<functional>

namespace vtr {
namespace pose_graph {
namespace eval {
namespace mask {
namespace variable {


template <class GRAPH>
class Eval : public BaseEval {
 public:
  PTR_TYPEDEFS(Eval);

  Eval(const GRAPH &graph, const std::function<ReturnType(const GRAPH&, const EdgeId&)> &edgeFcn, const std::function<ReturnType(const GRAPH&, const VertexId&)> &vertexFcn)
   : graph_(graph), eFcn(edgeFcn), vFcn(vertexFcn) {}

 protected:
  ReturnType computeEdge(const EdgeId &e) override {
    return eFcn(graph_, e);
  }

  ReturnType computeVertex(const VertexId &v) override {
    return vFcn(graph_, v);
  }

 private:
  const GRAPH &graph_;
  const std::function<ReturnType(const GRAPH&, const EdgeId&)> &eFcn;
  const std::function<ReturnType(const GRAPH&, const VertexId&)> &vFcn;
};

template <class GRAPH>
class CachedEval : public BaseCachedEval {
 public:
  PTR_TYPEDEFS(CachedEval);

  CachedEval(const GRAPH &graph, const std::function<ReturnType(const GRAPH&, const EdgeId&)> &edgeFcn, const std::function<ReturnType(const GRAPH&, const VertexId&)> &vertexFcn)
   : graph_(graph), eFcn(edgeFcn), vFcn(vertexFcn){}

 protected:
  ReturnType computeEdge(const EdgeId &e) override {
    return eFcn(graph_, e);
  }

  ReturnType computeVertex(const VertexId &v) override {
    return vFcn(graph_, v);
  }

 private:
  const GRAPH &graph_;
  const std::function<ReturnType(const GRAPH&, const EdgeId&)> &eFcn;
  const std::function<ReturnType(const GRAPH&, const VertexId&)> &vFcn;
};

template <class GRAPH>
class WindowedEval : public BaseWindowedEval {
 public:
  PTR_TYPEDEFS(WindowedEval);

  WindowedEval(const GRAPH &graph, const size_t &cache_size, const std::function<ReturnType(const GRAPH&, const EdgeId&)> edgeFcn, const std::function<ReturnType(const GRAPH&, const VertexId&)> vertexFcn)
      : BaseWindowedEval(cache_size), graph_(graph), 
        eFcn(edgeFcn), vFcn(vertexFcn) {}

 protected:
  ReturnType computeEdge(const EdgeId &e) override {
    return eFcn(graph_, e);
  }

  ReturnType computeVertex(const VertexId &v) override {
    return vFcn(graph_, v);
  }

 private:
  const GRAPH &graph_;
  const std::function<ReturnType(const GRAPH&, const EdgeId&)> &eFcn;
  const std::function<ReturnType(const GRAPH&, const VertexId&)> &vFcn;
};

}  // namespace variable
}  // namespace mask
}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr