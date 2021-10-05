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
 * \file common.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/evaluator_base/common.hpp>

#include <vtr_pose_graph/serializable/rc_graph.hpp>
#include <vtr_pose_graph/serializable/rc_graph_base.hpp>

namespace vtr {
namespace pose_graph {
namespace eval {
namespace Mask {

template <class GRAPH>
class TemporalDirect : public virtual Typed<GRAPH>::Direct {
 public:
  using Base = typename Typed<GRAPH>::Direct;
  using Base::graph_;

  PTR_TYPEDEFS(TemporalDirect);

  static Ptr MakeShared() { return Ptr(new TemporalDirect()); }

  TemporalDirect() {}
  virtual ~TemporalDirect() {}

  TemporalDirect(const TemporalDirect &) = default;
  TemporalDirect &operator=(const TemporalDirect &) = default;
  TemporalDirect(TemporalDirect &&) = default;
  TemporalDirect &operator=(TemporalDirect &&) = default;

 protected:
  ReturnType computeEdge(const typename Base::EdgePtr &e) const override {
    // Compute the norm of the linear component of the edge transform
    return e->isTemporal();
  }
  ReturnType computeVertex(const typename Base::VertexPtr &) const {
    // Garbage computation to avoid warnings; vertices do not have a "distance"
    return true;
  }
};

template <class GRAPH>
class TemporalCaching : public virtual Typed<GRAPH>::Caching,
                        public virtual TemporalDirect<GRAPH> {
 public:
  PTR_TYPEDEFS(TemporalCaching);

  static Ptr MakeShared() { return Ptr(new TemporalCaching()); }

  TemporalCaching() {}
  virtual ~TemporalCaching() {}

  TemporalCaching(const TemporalCaching &) = default;
  TemporalCaching &operator=(const TemporalCaching &) = default;
  TemporalCaching(TemporalCaching &&other)
      : Typed<GRAPH>::Caching(std::move(other)),
        TemporalDirect<GRAPH>(std::move(other)) {}
  TemporalCaching &operator=(TemporalCaching &&other) {
    Typed<GRAPH>::Caching::operator=(std::move(other));
    TemporalDirect<GRAPH>::operator=(std::move(other));
    return *this;
  }
};

template <class GRAPH>
class TemporalWindowed : public virtual Typed<GRAPH>::Windowed,
                         public virtual TemporalCaching<GRAPH> {
 public:
  PTR_TYPEDEFS(TemporalWindowed);

  static Ptr MakeShared(const size_t &cacheSize) {
    return Ptr(new TemporalWindowed(cacheSize));
  }

  TemporalWindowed(const size_t &cacheSize)
      : Typed<GRAPH>::Windowed(cacheSize) {}
  virtual ~TemporalWindowed() {}

  TemporalWindowed(const TemporalWindowed &) = default;
  TemporalWindowed &operator=(const TemporalWindowed &) = default;
  TemporalWindowed(TemporalWindowed &&other)
      : Typed<GRAPH>::Windowed(std::move(other)),
        TemporalCaching<GRAPH>(std::move(other)) {}
  TemporalWindowed &operator=(TemporalWindowed &&other) {
    Typed<GRAPH>::Windowed::operator=(std::move(other));
    TemporalCaching<GRAPH>::operator=(std::move(other));
    return *this;
  }
};

template <class GRAPH>
struct Temporal {
  using Direct = TemporalDirect<GRAPH>;
  using Caching = TemporalCaching<GRAPH>;
  using Windowed = TemporalWindowed<GRAPH>;
};

}  // namespace Mask

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr
