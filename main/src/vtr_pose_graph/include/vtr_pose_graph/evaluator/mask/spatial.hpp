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
class SpatialDirect : public virtual Typed<GRAPH>::Direct {
 public:
  using Base = typename Typed<GRAPH>::Direct;
  using Base::graph_;

  PTR_TYPEDEFS(SpatialDirect);

  static Ptr MakeShared() { return Ptr(new SpatialDirect()); }

  SpatialDirect() {}
  virtual ~SpatialDirect() {}

  SpatialDirect(const SpatialDirect &) = default;
  SpatialDirect &operator=(const SpatialDirect &) = default;
  SpatialDirect(SpatialDirect &&) = default;
  SpatialDirect &operator=(SpatialDirect &&) = default;

 protected:
  ReturnType computeEdge(const typename Base::EdgePtr &e) const override {
    // Compute the norm of the linear component of the edge transform
    return e->isSpatial();
  }
  ReturnType computeVertex(const typename Base::VertexPtr &) const {
    // Garbage computation to avoid warnings; vertices do not have a "distance"
    return true;
  }
};

template <class GRAPH>
class SpatialCaching : public virtual Typed<GRAPH>::Caching,
                       public virtual SpatialDirect<GRAPH> {
 public:
  PTR_TYPEDEFS(SpatialCaching);

  static Ptr MakeShared() { return Ptr(new SpatialCaching()); }

  SpatialCaching() {}
  virtual ~SpatialCaching() {}

  SpatialCaching(const SpatialCaching &) = default;
  SpatialCaching &operator=(const SpatialCaching &) = default;
  SpatialCaching(SpatialCaching &&other)
      : Typed<GRAPH>::Caching(std::move(other)),
        SpatialDirect<GRAPH>(std::move(other)) {}
  SpatialCaching &operator=(SpatialCaching &&other) {
    Typed<GRAPH>::Caching::operator=(std::move(other));
    SpatialDirect<GRAPH>::operator=(std::move(other));
    return *this;
  }
};

template <class GRAPH>
class SpatialWindowed : public virtual Typed<GRAPH>::Windowed,
                        public virtual SpatialCaching<GRAPH> {
 public:
  PTR_TYPEDEFS(SpatialWindowed);

  static Ptr MakeShared(const size_t &cacheSize) {
    return Ptr(new SpatialWindowed(cacheSize));
  }

  SpatialWindowed(const size_t &cacheSize)
      : Typed<GRAPH>::Windowed(cacheSize) {}
  virtual ~SpatialWindowed() {}

  SpatialWindowed(const SpatialWindowed &) = default;
  SpatialWindowed &operator=(const SpatialWindowed &) = default;
  SpatialWindowed(SpatialWindowed &&other)
      : Typed<GRAPH>::Windowed(std::move(other)),
        SpatialCaching<GRAPH>(std::move(other)) {}
  SpatialWindowed &operator=(SpatialWindowed &&other) {
    Typed<GRAPH>::Windowed::operator=(std::move(other));
    SpatialCaching<GRAPH>::operator=(std::move(other));
    return *this;
  }
};

template <class GRAPH>
struct Spatial {
  using Direct = SpatialDirect<GRAPH>;
  using Caching = SpatialCaching<GRAPH>;
  using Windowed = SpatialWindowed<GRAPH>;
};

}  // namespace Mask

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr
