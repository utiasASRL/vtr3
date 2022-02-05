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
 * \file types.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <memory>

#include "vtr_pose_graph/evaluator/evaluators.hpp"
#include "vtr_pose_graph/path/localization_chain.hpp"
#include "vtr_pose_graph/serializable/rc_graph.hpp"

#include "vtr_tactic_msgs/msg/env_info.hpp"

namespace vtr {
namespace tactic {

/// storage types
using Timestamp = storage::Timestamp;

/// pose graph structures
using Graph = pose_graph::RCGraph;
using GraphBase = Graph::Base;
using VertexId = pose_graph::VertexId;
using Vertex = pose_graph::RCVertex;
using EdgeId = pose_graph::EdgeId;
using Edge = pose_graph::RCEdge;
using EdgeType = pose_graph::EdgeType;
using EdgeTransform = pose_graph::EdgeTransform;

template <class GraphT>
using PrivilegedEvaluator = pose_graph::eval::mask::privileged::Eval<GraphT>;
template <class GraphT>
using TemporalEvaluator = pose_graph::eval::mask::temporal::Eval<GraphT>;
using LocalizationChain = pose_graph::LocalizationChain<pose_graph::RCGraph>;

/// mission planning
using PathType = VertexId::Vector;

/// tactic types
using EnvInfo = vtr_tactic_msgs::msg::EnvInfo;

/** \brief Defines the possible pipeline types to be used by tactics */
enum class PipelineMode : uint8_t {
  Idle,             // Idle
  TeachMetricLoc,   // Teach - Metric localization
  TeachBranch,      // Teach - branching from existing path
  TeachMerge,       // Teach - merging into existing path
  RepeatMetricLoc,  //
  RepeatFollow,     // Repeat - path following
};
std::ostream& operator<<(std::ostream& os, const PipelineMode& signal);

/** \brief the vertex creation test result */
enum class KeyframeTestResult : int {
  CREATE_VERTEX = 0,
  CREATE_CANDIDATE = 1,
  FAILURE = 2,
  DO_NOTHING = 3
};

/** \brief Full metric and topological localization in one package */
struct Localization {
  Localization(const VertexId& vertex = VertexId::Invalid(),
               const EdgeTransform& T_robot_vertex = EdgeTransform(true),
               bool has_localized = false)
      : v(vertex), T(T_robot_vertex), localized(has_localized) {}
  Timestamp stamp = -1;
  VertexId v;
  EdgeTransform T;
  bool localized;
};

}  // namespace tactic
}  // namespace vtr