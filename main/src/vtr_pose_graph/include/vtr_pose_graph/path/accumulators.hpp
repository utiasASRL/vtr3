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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_pose_graph/index/graph_base.hpp"

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

template <class RVAL, class GRAPH, class ITER = typename GRAPH::OrderedIter>
struct ComposeTf : public AccumulatorBase<RVAL, GRAPH, ITER> {
  RVAL operator()(const RVAL& val, const ITER& it) const {
    // T_root_current = T_root_prev * T_prev_current
    return val * it->T();
  }
};

template <class RVAL, class ITER, class GRAPH = typename ITER::GraphType>
RVAL ComposeTfAccumulator(const ITER& first, const ITER& last,
                          const RVAL& val) {
  return accumulator(first, last, val, ComposeTf<RVAL, GRAPH, ITER>());
}

template <class RVAL, class GRAPH, class ITER = typename GRAPH::OrderedIter>
struct ComputePathDistance : public AccumulatorBase<RVAL, GRAPH, ITER> {
  RVAL operator()(const RVAL& val, const ITER& it) const {
    // Path distance between two vertices is the norm of the translational
    // component of the transformation.
    return val + it->T().r_ab_inb().norm();
  }
};

template <class RVAL, class ITER, class GRAPH = typename ITER::GraphType>
RVAL ComputePathDistanceAccumulator(const ITER& first, const ITER& last,
                                    const RVAL& val) {
  return accumulator(first, last, val,
                     ComputePathDistance<RVAL, GRAPH, ITER>());
}

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr
