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
 * \file accumulators.hpp
 * \brief Implements binary accumulators that can be used with accumulate,
 * and applied to a graph using ordered iterators.
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/index/graph_base.hpp>

namespace vtr {
namespace pose_graph {
namespace eval {

//////////////////////////////
/// Accumulator Definition ///
//////////////////////////////

/// This is basically std::accumulate, without the dereference
template <class It, class T, class Op>
T accumulator(const It& first, const It& last, T val, Op op) {
  for (It it = first; it != last; ++it) val = op(val, it);
  return val;
}

////////////////////////////
/// Op Class Definitions ///
////////////////////////////

/// The base class for Edge operators
template <class RVAL, class GRAPH, class ITER = typename GRAPH::OrderedIter>
struct AccumulatorBase {
  virtual RVAL operator()(const RVAL& val, const ITER& it) const = 0;
};

#define BINARY_EDGE_OP(Name)                                                   \
  template <class RVAL, class GRAPH, class ITER = typename GRAPH::OrderedIter> \
  struct Name : public AccumulatorBase<RVAL, GRAPH, ITER>

#define BINARY_EDGE_OP_EVAL                         \
  using Rval = RVAL;                                \
  using Iterator = ITER;                            \
  using VertexType = typename GRAPH::VertexType;    \
  using VertexIdType = typename VertexType::IdType; \
  using EdgeType = typename GRAPH::EdgeType;        \
  Rval operator()(const Rval& val, const Iterator& it) const

#define BINARY_EDGE_ACCUMULATOR(Name)                                       \
  template <class RVAL, class ITER, class GRAPH = typename ITER::GraphType> \
  RVAL Name##Accumulator(const ITER& first, const ITER& last,               \
                         const RVAL& val) {                                 \
    return accumulator(first, last, val, Name<RVAL, GRAPH, ITER>());        \
  }

// Gives back T_begin_end, but only if it's iterating over a simple chain from
// begin to end
// clang-format off
BINARY_EDGE_OP(ComposeTf){
  BINARY_EDGE_OP_EVAL{// T_root_current = T_root_prev * T_prev_current
    return val * it->T();
  }
};
// clang-format on
BINARY_EDGE_ACCUMULATOR(ComposeTf);

// Gives back the path distance between the start and end of a simple chain from
// begin to end
// clang-format off
BINARY_EDGE_OP(ComputePathDistance){
  BINARY_EDGE_OP_EVAL{// Path distance between two vertices is the norm of the
                      // translational component
                      // of the transformation.
    return val + it->T().r_ab_inb().norm();
  }
};
// clang-format on
BINARY_EDGE_ACCUMULATOR(ComputePathDistance);
}
}
}
