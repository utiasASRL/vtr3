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
namespace Weight {

template <class GRAPH>
class AngleDirect : public virtual Typed<GRAPH>::Direct {
 public:
  using Base = typename Typed<GRAPH>::Direct;
  using Base::graph_;

  PTR_TYPEDEFS(AngleDirect);

  static Ptr MakeShared() { return Ptr(new AngleDirect()); }

  AngleDirect() {}
  virtual ~AngleDirect() {}

  AngleDirect(const AngleDirect &) = default;
  AngleDirect &operator=(const AngleDirect &) = default;
  AngleDirect(AngleDirect &&) = default;
  AngleDirect &operator=(AngleDirect &&) = default;

 protected:
  ReturnType computeEdge(const typename Base::EdgePtr &e) const override {
    // Compute the norm of the linear component of the edge transform
    return e->T().vec().template tail<3>().norm();
  }
  ReturnType computeVertex(const typename Base::VertexPtr &) const {
    // Garbage computation to avoid warnings; vertices do not have a "distance"
    return 0.f;
  }
};

template <class GRAPH>
class AngleCaching : public virtual Typed<GRAPH>::Caching,
                     public virtual AngleDirect<GRAPH> {
 public:
  PTR_TYPEDEFS(AngleCaching);

  static Ptr MakeShared() { return Ptr(new AngleCaching()); }

  AngleCaching() {}
  virtual ~AngleCaching() {}

  AngleCaching(const AngleCaching &) = default;
  AngleCaching &operator=(const AngleCaching &) = default;
  AngleCaching(AngleCaching &&other)
      : Typed<GRAPH>::Caching(std::move(other)),
        AngleDirect<GRAPH>(std::move(other)) {}
  AngleCaching &operator=(AngleCaching &&other) {
    Typed<GRAPH>::Caching::operator=(std::move(other));
    AngleDirect<GRAPH>::operator=(std::move(other));
    return *this;
  }
};

template <class GRAPH>
class AngleWindowed : public virtual Typed<GRAPH>::Windowed,
                      public virtual AngleCaching<GRAPH> {
 public:
  PTR_TYPEDEFS(AngleWindowed);

  static Ptr MakeShared(const size_t &cacheSize) {
    return Ptr(new AngleWindowed(cacheSize));
  }

  AngleWindowed(const size_t &cacheSize) : Typed<GRAPH>::Windowed(cacheSize) {}
  virtual ~AngleWindowed() {}

  AngleWindowed(const AngleWindowed &) = default;
  AngleWindowed &operator=(const AngleWindowed &) = default;
  AngleWindowed(AngleWindowed &&other)
      : Typed<GRAPH>::Windowed(std::move(other)),
        AngleCaching<GRAPH>(std::move(other)) {}
  AngleWindowed &operator=(AngleWindowed &&other) {
    Typed<GRAPH>::Windowed::operator=(std::move(other));
    AngleCaching<GRAPH>::operator=(std::move(other));
    return *this;
  }
};

template <class GRAPH>
struct Angle {
  using Direct = AngleDirect<GRAPH>;
  using Caching = AngleCaching<GRAPH>;
  using Windowed = AngleWindowed<GRAPH>;
};

}  // namespace Weight

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr
