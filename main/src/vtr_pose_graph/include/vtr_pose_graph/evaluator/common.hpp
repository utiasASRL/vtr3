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

#include <vtr_pose_graph/evaluator/mask_evaluator.hpp>
#include <vtr_pose_graph/evaluator/weight_evaluator.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>

namespace vtr {
namespace pose_graph {
namespace eval {
namespace Weight {

/// \brief Evaluator for computing edge distance
EVAL_SIMPLE_DEFINE(Distance);

EVAL_SIMPLE_COMPUTE_EDGE(Distance) const {
  // Compute the norm of the linear component of the edge transform
  return e->T().r_ab_inb().norm();
}

EVAL_SIMPLE_COMPUTE_VERTEX(Distance) const {
  // Garbage computation to avoid warnings; vertices do not have a "distance"
  (void)&v;
  return 0.f;
}

/// \brief Evaluator for computing edge distance
EVAL_SIMPLE_DEFINE(Temporal)

EVAL_SIMPLE_COMPUTE_EDGE(Temporal) const {
  // Each temporal edge is weighted 1.f
  return e->isTemporal() ? 1.f : 0.f;
}

EVAL_SIMPLE_COMPUTE_VERTEX(Temporal) const {
  // Garbage computation to avoid warnings; vertices do not have a "distance"
  (void)&v;
  return 0.f;
}

/// \brief Evaluator for computer edge rotation angle
EVAL_SIMPLE_DEFINE(Angle)

EVAL_SIMPLE_COMPUTE_EDGE(Angle) const {
  // Compute the norm of the linear component of the edge transform
  return e->T().vec().template tail<3>().norm();
}

EVAL_SIMPLE_COMPUTE_VERTEX(Angle) const {
  // Garbage computation to avoid warnings; vertices do not have an "angle"
  (void)&v;
  return 0.f;
}

/// \brief Evaluator for computing a weighted sum of edge distance and rotation
/// angle
DIRECT_EVAL(DistAngle) {
 public:
  DIRECT_PREAMBLE(DistAngle)
  EVAL_DESTRUCTOR(DistAngle, Direct) {}
  EVAL_CONSTRUCTOR(DistAngle, Direct, (double angleWeight = 1.0), (angleWeight))
      : angleWeight_(angleWeight) {}

 protected:
  EVAL_COMPUTE_EDGE const {
    auto tmp = e->T().vec();
    return tmp.template head<3>().norm() +
           angleWeight_ * tmp.template tail<3>().norm();
  }

  EVAL_COMPUTE_VERTEX const {
    (void)&v;
    return 0.f;
  }

  double angleWeight_;
};

CACHING_EVAL(DistAngle) {
 public:
  CACHING_PREAMBLE(DistAngle);
  EVAL_DESTRUCTOR(DistAngle, Caching) {}
  EVAL_CONSTRUCTOR(DistAngle, Caching, (double angleWeight = 1.0),
                   (angleWeight))
      : DirectBase(angleWeight) {}
};

WINDOWED_EVAL(DistAngle) {
 public:
  WINDOWED_PREAMBLE(DistAngle);
  EVAL_DESTRUCTOR(DistAngle, Windowed) {}

  EVAL_CONSTRUCTOR(DistAngle, Windowed, (double angleWeight = 1.0),
                   (angleWeight))
      : DirectBase(angleWeight) {}
  EVAL_CONSTRUCTOR(DistAngle, Windowed, (size_t N), (N)) : AbstractBase(N) {}
  EVAL_CONSTRUCTOR(DistAngle, Windowed, (double angleWeight, size_t N),
                   (angleWeight, N))
      : AbstractBase(N), DirectBase(angleWeight) {}
};

EVAL_TYPEDEFS(DistAngle)

// extern
EVAL_EXPLICIT_DECLARE(Distance, RCGraph)
EVAL_EXPLICIT_DECLARE(Distance, RCGraphBase)
EVAL_EXPLICIT_DECLARE(Distance, BasicGraph)
EVAL_EXPLICIT_DECLARE(Distance, BasicGraphBase)

EVAL_EXPLICIT_DECLARE(Temporal, RCGraph)
EVAL_EXPLICIT_DECLARE(Temporal, RCGraphBase)
EVAL_EXPLICIT_DECLARE(Temporal, BasicGraph)
EVAL_EXPLICIT_DECLARE(Temporal, BasicGraphBase)

EVAL_EXPLICIT_DECLARE(Angle, RCGraph)
EVAL_EXPLICIT_DECLARE(Angle, RCGraphBase)
EVAL_EXPLICIT_DECLARE(Angle, BasicGraph)
EVAL_EXPLICIT_DECLARE(Angle, BasicGraphBase)

EVAL_EXPLICIT_DECLARE(DistAngle, RCGraph)
EVAL_EXPLICIT_DECLARE(DistAngle, RCGraphBase)
EVAL_EXPLICIT_DECLARE(DistAngle, BasicGraph)
EVAL_EXPLICIT_DECLARE(DistAngle, BasicGraphBase)

}  // namespace Weight

namespace Mask {

/// \brief Evaluator for selecting privileged edges
EVAL_SIMPLE_RECURSIVE_DEFINE(Privileged)

EVAL_SIMPLE_COMPUTE_EDGE(Privileged) {
  // Return true if the edge is manual, or the edge is between two privileged
  // vertices (computed recursively)
  return e->isManual() || ((*this)[e->from()] && (*this)[e->to()]);
}

EVAL_SIMPLE_COMPUTE_VERTEX(Privileged) {
  typedef typename Base::EdgeIdType::Base::Type EidEnumType;
  // Check to see if any of the connected edges are manual edges
  for (uint32_t i = 0; i < Base::EdgeIdType::Base::NumTypes(); ++i) {
    for (const typename Base::VertexIdType& nvid :
         v->neighbours(EidEnumType(i))) {
      try {
        if (graph_->at(v->id(), nvid)->isManual()) return true;
      } catch (const std::out_of_range& e) {
      }
    }
  }
  return false;
}

// TODO: This is going to involve a lot of stupid repetition if called in a
// const scope...
EVAL_SIMPLE_COMPUTE_EDGE(Privileged) const {
  // Return true if the edge is manual, or the edge is between two privileged
  // vertices (computed recursively)
  return e->isManual() || (this->at(e->from()) && this->at(e->to()));
}

EVAL_SIMPLE_COMPUTE_VERTEX(Privileged) const {
  typedef typename Base::EdgeIdType::Base::Type EidEnumType;
  // Check to see if any of the connected edges are manual edges
  for (uint32_t i = 0; i < Base::EdgeIdType::Base::NumTypes(); ++i) {
    for (const typename Base::VertexIdType& nvid :
         v->neighbours(EidEnumType(i))) {
      try {
        if (graph_->at(v->id(), nvid)->isManual()) return true;
      } catch (const std::out_of_range& e) {
      }
    }
  }
  return false;
}

EVAL_SIMPLE_DEFINE(Spatial)

EVAL_SIMPLE_COMPUTE_EDGE(Spatial) const { return e->isSpatial(); }

EVAL_SIMPLE_COMPUTE_VERTEX(Spatial) const {
  // Garbage computation to avoid warnings; vertices do not have a "distance"
  (void)&v;
  return true;
}

EVAL_SIMPLE_DEFINE(SimpleTemporal)

EVAL_SIMPLE_COMPUTE_EDGE(SimpleTemporal) const { return e->isTemporal(); }

EVAL_SIMPLE_COMPUTE_VERTEX(SimpleTemporal) const {
  // Garbage computation to avoid warnings; vertices do not have a "distance"
  (void)&v;
  return true;
}

/// \brief Evaluator for deciding if a vertex is ahead/behind of a root vertex
DIRECT_EVAL(DirectionFromVertex) {
 public:
  DIRECT_PREAMBLE(DirectionFromVertex)
  EVAL_DESTRUCTOR(DirectionFromVertex, Direct) {}
  EVAL_CONSTRUCTOR(DirectionFromVertex, Direct,
                   (VertexId id, bool reverse = false), (id, reverse))
      : reverse_(reverse), id_(id) {}

 protected:
  EVAL_COMPUTE_EDGE const override {
    // Garbage computation to avoid warnings
    (void)&e;
    return true;
  }

  EVAL_COMPUTE_VERTEX const override {
    const auto& id = v->id();
    // We can't tell direction if the majors are different
    if (id_.majorId() != id.majorId()) return true;
    // Itself is in the mask
    if (id_.minorId() == id.minorId()) return true;
    // If the query is earlier, and we're headed in reverse, OK.
    if ((id_.minorId() > id.minorId()) == reverse_) return true;
    // Nope!
    return false;
  }

  const bool reverse_;
  const VertexId id_;
};

// extern
EVAL_EXPLICIT_DECLARE(Privileged, RCGraph)
EVAL_EXPLICIT_DECLARE(Privileged, RCGraphBase)
EVAL_EXPLICIT_DECLARE(Privileged, BasicGraph)
EVAL_EXPLICIT_DECLARE(Privileged, BasicGraphBase)

EVAL_EXPLICIT_DECLARE(Spatial, RCGraph)
EVAL_EXPLICIT_DECLARE(Spatial, RCGraphBase)
EVAL_EXPLICIT_DECLARE(Spatial, BasicGraph)
EVAL_EXPLICIT_DECLARE(Spatial, BasicGraphBase)

EVAL_EXPLICIT_DECLARE(SimpleTemporal, RCGraph)
EVAL_EXPLICIT_DECLARE(SimpleTemporal, RCGraphBase)
EVAL_EXPLICIT_DECLARE(SimpleTemporal, BasicGraph)
EVAL_EXPLICIT_DECLARE(SimpleTemporal, BasicGraphBase)

//   DistanceFromVertex is a special flower, because it doesn't have Caching or
//   Windowed variants
extern template class DirectionFromVertexDirect<RCGraph>;
extern template class DirectionFromVertexDirect<RCGraphBase>;
extern template class DirectionFromVertexDirect<BasicGraph>;
extern template class DirectionFromVertexDirect<BasicGraphBase>;

}  // namespace Mask

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr
