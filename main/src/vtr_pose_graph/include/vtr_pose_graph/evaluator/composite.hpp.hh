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
 * \file composite.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/evaluator/evaluator_base.hpp>
#include <vtr_pose_graph/index/composite_graph.hpp>

namespace vtr {
namespace pose_graph {
namespace eval {

template <class RVAL, class GRAPH>
class CompositeEval : public virtual DirectBase<RVAL, CompositeGraph<GRAPH>> {
 public:
  using Base = DirectBase<RVAL, CompositeGraph<GRAPH>>;
  using BasePtr = typename EvalBase<RVAL>::Ptr;

  using GraphPtr = typename CompositeGraph<GRAPH>::Ptr;
  using EdgePtr = typename CompositeGraph<GRAPH>::EdgePtr;
  using VertexPtr = typename CompositeGraph<GRAPH>::VertexPtr;

  using EdgeIdType = typename GRAPH::EdgeIdType;
  using VertexIdType = typename GRAPH::VertexIdType;

  PTR_TYPEDEFS(CompositeEval);

  CompositeEval(const CompositeGraph<GRAPH>* graph, const BasePtr& eval);
  CompositeEval(const CompositeEval&) = default;
  CompositeEval(CompositeEval&&) = default;

  virtual ~CompositeEval() {}

  CompositeEval& operator=(const CompositeEval&) = default;
  CompositeEval& operator=(CompositeEval&&) = default;

 protected:
  BasePtr eval_;

  virtual RVAL computeEdge(const EdgePtr& e) const;
  virtual RVAL computeVertex(const VertexPtr& v) const;

  virtual RVAL computeEdge(const EdgePtr& e);
  virtual RVAL computeVertex(const VertexPtr& v);
};

template <class RVAL, class GRAPH>
CompositeEval<RVAL, GRAPH>::CompositeEval(const CompositeGraph<GRAPH>* graph,
                                          const BasePtr& eval)
    : Base(), eval_(eval) {
  this->setGraph((void*)graph);
  eval_->setGraph(graph->base().get());
}

template <class RVAL, class GRAPH>
RVAL CompositeEval<RVAL, GRAPH>::computeEdge(const EdgePtr& e) const {
  auto res = RVAL();
  auto it = ++e->begin();
  for (; it != e->end(); ++it) {
    res += this->eval_->at(EdgeIdType(*it));
  }
  return res;
}

template <class RVAL, class GRAPH>
RVAL CompositeEval<RVAL, GRAPH>::computeVertex(const VertexPtr& v) const {
  return this->eval_->at(v->id());
}

template <class RVAL, class GRAPH>
RVAL CompositeEval<RVAL, GRAPH>::computeEdge(const EdgePtr& e) {
  auto res = RVAL();
  auto it = ++e->begin();
  for (; it != e->end(); ++it) {
    res += this->eval_->operator[](EdgeIdType(*it));
  }
  return res;
}

template <class RVAL, class GRAPH>
RVAL CompositeEval<RVAL, GRAPH>::computeVertex(const VertexPtr& v) {
  return this->eval_->operator[](v->id());
}

template <class RVAL, class GRAPH>
class CompositeEvalCaching
    : public virtual CachingBase<RVAL, CompositeGraph<GRAPH>>,
      public virtual CompositeEval<RVAL, GRAPH> {
 public:
  PTR_TYPEDEFS(CompositeEvalCaching);

  using Base = CachingBase<RVAL, CompositeGraph<GRAPH>>;
  using CompositeBase = CompositeEval<RVAL, GRAPH>;
  using BasePtr = typename EvalBase<RVAL>::Ptr;
  using GraphPtr = typename CompositeBase::GraphPtr;

  static Ptr MakeShared(const CompositeGraph<GRAPH>* graph,
                        const BasePtr& eval) {
    return Ptr(new CompositeEvalCaching(graph, eval));
  }

  CompositeEvalCaching(const CompositeGraph<GRAPH>* graph, const BasePtr& eval)
      : CompositeBase(graph, eval) {}
  CompositeEvalCaching(const CompositeEvalCaching&) = default;

  virtual ~CompositeEvalCaching() {}

  CompositeEvalCaching& operator=(const CompositeEvalCaching&) = default;

  EXPLICIT_VIRTUAL_MOVE(CompositeEvalCaching, Base, CompositeBase)
};

template <class RVAL, class GRAPH>
class CompositeEvalWindowed
    : public virtual WindowedBase<RVAL, CompositeGraph<GRAPH>>,
      public virtual CompositeEvalCaching<RVAL, GRAPH> {
 public:
  PTR_TYPEDEFS(CompositeEvalWindowed);

  CompositeEvalWindowed(const CompositeEvalWindowed&) = default;
  CompositeEvalWindowed& operator=(const CompositeEvalWindowed&) = default;

  using Base = WindowedBase<RVAL, CompositeGraph<GRAPH>>;
  using CompositeBase = CompositeEvalCaching<RVAL, GRAPH>;
  using BasePtr = typename EvalBase<RVAL>::Ptr;
  using GraphPtr = typename CompositeBase::GraphPtr;

  static Ptr MakeShared(const CompositeGraph<GRAPH>* graph, const BasePtr& eval,
                        const size_t& cacheSize) {
    return Ptr(new CompositeEvalWindowed(graph, eval, cacheSize));
  }

  CompositeEvalWindowed(const CompositeGraph<GRAPH>* graph, const BasePtr& eval,
                        const size_t& cacheSize)
      : Base(cacheSize), CompositeBase(graph, eval) {}

  virtual ~CompositeEvalWindowed() {}

  EXPLICIT_VIRTUAL_MOVE(CompositeEvalWindowed, Base, CompositeBase)
};

template <class EVAL>
struct Composite {
  using rtype = decltype(std::declval<EVAL>().at(0));
  using BasePtr = typename EvalBase<rtype>::Ptr;

  template <typename GRAPH, typename... Args>
  static BasePtr MakeDirect(const CompositeGraph<GRAPH>* graph,
                            Args&&... args) {
    BasePtr eval(new EVAL(std::forward<Args>(args)...));
    return BasePtr(new CompositeEval<rtype, GRAPH>(graph, eval));
  }

  template <typename GRAPH, typename... Args>
  static BasePtr MakeCaching(const CompositeGraph<GRAPH>* graph,
                             Args&&... args) {
    BasePtr eval(new EVAL(std::forward<Args>(args)...));
    return BasePtr(new CompositeEvalCaching<rtype, GRAPH>(graph, eval));
  }

  template <typename GRAPH, typename... Args>
  static BasePtr MakeWindowed(const CompositeGraph<GRAPH>* graph,
                              const size_t& cacheSize, Args&&... args) {
    BasePtr eval(new EVAL(std::forward<Args>(args)...));
    return BasePtr(
        new CompositeEvalWindowed<rtype, GRAPH>(graph, eval, cacheSize));
  }
};

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr