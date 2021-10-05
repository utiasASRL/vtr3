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
class DistanceDirect : public virtual Typed<GRAPH>::Direct {
 public:
  using Base = typename Typed<GRAPH>::Direct;
  using Base::graph_;

  PTR_TYPEDEFS(DistanceDirect);

  static Ptr MakeShared() { return Ptr(new DistanceDirect()); }

  DistanceDirect() {}
  virtual ~DistanceDirect() {}

  DistanceDirect(const DistanceDirect &) = default;
  DistanceDirect &operator=(const DistanceDirect &) = default;
  DistanceDirect(DistanceDirect &&) = default;
  DistanceDirect &operator=(DistanceDirect &&) = default;

 protected:
  ReturnType computeEdge(const typename Base::EdgePtr &e) const override {
    // Compute the norm of the linear component of the edge transform
    return e->T().r_ab_inb().norm();
  }
  ReturnType computeVertex(const typename Base::VertexPtr &) const {
    // Garbage computation to avoid warnings; vertices do not have a "distance"
    return 0.f;
  }
};

template <class GRAPH>
class DistanceCaching : public virtual Typed<GRAPH>::Caching,
                        public virtual DistanceDirect<GRAPH> {
 public:
  PTR_TYPEDEFS(DistanceCaching);

  static Ptr MakeShared() { return Ptr(new DistanceCaching()); }

  DistanceCaching() {}
  virtual ~DistanceCaching() {}

  DistanceCaching(const DistanceCaching &) = default;
  DistanceCaching &operator=(const DistanceCaching &) = default;
  DistanceCaching(DistanceCaching &&other)
      : Typed<GRAPH>::Caching(std::move(other)),
        DistanceDirect<GRAPH>(std::move(other)) {}
  DistanceCaching &operator=(DistanceCaching &&other) {
    Typed<GRAPH>::Caching::operator=(std::move(other));
    DistanceDirect<GRAPH>::operator=(std::move(other));
    return *this;
  }
};

template <class GRAPH>
class DistanceWindowed : public virtual Typed<GRAPH>::Windowed,
                         public virtual DistanceCaching<GRAPH> {
 public:
  PTR_TYPEDEFS(DistanceWindowed);

  static Ptr MakeShared(const size_t &cacheSize) {
    return Ptr(new DistanceWindowed(cacheSize));
  }

  DistanceWindowed(const size_t &cacheSize)
      : Typed<GRAPH>::Windowed(cacheSize) {}
  virtual ~DistanceWindowed() {}

  DistanceWindowed(const DistanceWindowed &) = default;
  DistanceWindowed &operator=(const DistanceWindowed &) = default;
  DistanceWindowed(DistanceWindowed &&other)
      : Typed<GRAPH>::Windowed(std::move(other)),
        DistanceCaching<GRAPH>(std::move(other)) {}
  DistanceWindowed &operator=(DistanceWindowed &&other) {
    Typed<GRAPH>::Windowed::operator=(std::move(other));
    DistanceCaching<GRAPH>::operator=(std::move(other));
    return *this;
  }
};

template <class GRAPH>
struct Distance {
  using Direct = DistanceDirect<GRAPH>;
  using Caching = DistanceCaching<GRAPH>;
  using Windowed = DistanceWindowed<GRAPH>;
};

}  // namespace Weight

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr
