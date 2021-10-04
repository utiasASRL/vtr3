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
class DistAngleDirect : public virtual Typed<GRAPH>::Direct {
 public:
  using AbstractBase = typename Typed<GRAPH>::Direct;
  PTR_TYPEDEFS(DistAngleDirect);

  static Ptr MakeShared(double angleWeight = 1.0) {
    return Ptr(new DistAngleDirect(angleWeight));
  }

  DistAngleDirect(double angleWeight = 1.0) : angleWeight_(angleWeight) {}

  virtual ~DistAngleDirect() {}

  DistAngleDirect(const DistAngleDirect &) = default;
  DistAngleDirect &operator=(const DistAngleDirect &) = default;
  DistAngleDirect(DistAngleDirect &&) = default;
  DistAngleDirect &operator=(DistAngleDirect &&) = default;

 protected:
  ReturnType computeEdge(
      const typename Typed<GRAPH>::Direct::EdgePtr &e) const override {
    auto tmp = e->T().vec();
    return tmp.template head<3>().norm() +
           angleWeight_ * tmp.template tail<3>().norm();
  }

  ReturnType computeVertex(
      const typename Typed<GRAPH>::Direct::VertexPtr &) const override {
    return 0.f;
  }

  double angleWeight_;
};

template <class GRAPH>
class DistAngleCaching : public virtual Typed<GRAPH>::Caching,
                         public virtual DistAngleDirect<GRAPH> {
 public:
  using AbstractBase = typename Typed<GRAPH>::Caching;
  using DirectBase = DistAngleDirect<GRAPH>;
  PTR_TYPEDEFS(DistAngleCaching);

  static Ptr MakeShared(double angleWeight = 1.0) {
    return Ptr(new DistAngleCaching(angleWeight));
  }
  DistAngleCaching(double angleWeight = 1.0) : DirectBase(angleWeight) {}

  virtual ~DistAngleCaching() {}

  DistAngleCaching(const DistAngleCaching &) = default;
  DistAngleCaching &operator=(const DistAngleCaching &) = default;
  DistAngleCaching(DistAngleCaching &&other)
      : AbstractBase(std::move(other)), DirectBase(std::move(other)) {}
  DistAngleCaching &operator=(DistAngleCaching &&other) {
    AbstractBase::operator=(std::move(other));
    DirectBase::operator=(std::move(other));
    return *this;
  }
};

template <class GRAPH>
class DistAngleWindowed : public virtual Typed<GRAPH>::Windowed,
                          public virtual DistAngleCaching<GRAPH> {
 public:
  using AbstractBase = typename Typed<GRAPH>::Windowed;
  using DirectBase = DistAngleDirect<GRAPH>;
  using CachingBase = DistAngleCaching<GRAPH>;
  PTR_TYPEDEFS(DistAngleWindowed);

  static Ptr MakeShared(double angleWeight = 1.0) {
    return Ptr(new DistAngleWindowed(angleWeight));
  }
  DistAngleWindowed(double angleWeight = 1.0) : DirectBase(angleWeight) {}

  static Ptr MakeShared(size_t N) { return Ptr(new DistAngleWindowed(N)); }
  DistAngleWindowed(size_t N) : AbstractBase(N) {}

  static Ptr MakeShared(double angleWeight, size_t N) {
    return Ptr(new DistAngleWindowed(angleWeight, N));
  }
  DistAngleWindowed(double angleWeight, size_t N)
      : AbstractBase(N), DirectBase(angleWeight) {}

  virtual ~DistAngleWindowed() {}

  DistAngleWindowed(const DistAngleWindowed &) = default;
  DistAngleWindowed &operator=(const DistAngleWindowed &) = default;
  DistAngleWindowed(DistAngleWindowed &&other)
      : AbstractBase(std::move(other)), DirectBase(std::move(other)) {}
  DistAngleWindowed &operator=(DistAngleWindowed &&other) {
    AbstractBase::operator=(std::move(other));
    DirectBase::operator=(std::move(other));
    return *this;
  }
};

template <class GRAPH>
struct DistAngle {
  using Direct = DistAngleDirect<GRAPH>;
  using Caching = DistAngleCaching<GRAPH>;
  using Windowed = DistAngleWindowed<GRAPH>;
};

}  // namespace Weight

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr
