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
using GraphBase = pose_graph::RCGraphBase;
using RunId = pose_graph::RCRun::IdType;
using RunIdSet = std::set<RunId>;
using Vertex = pose_graph::RCVertex;
using VertexId = pose_graph::VertexId;
using EdgeId = pose_graph::EdgeId;
using EdgeTransform = pose_graph::RCEdge::TransformType;
using PersistentId = pose_graph::RCVertex::PersistentIdType;

template <class GraphT>
using PrivilegedEvaluator = pose_graph::eval::Mask::PrivilegedDirect<GraphT>;
template <class GraphT>
using TemporalEvaluator = pose_graph::eval::Mask::TemporalDirect<GraphT>;
using LocalizationChain = pose_graph::LocalizationChain<pose_graph::RCGraph>;

/// mission planning
using PathType = pose_graph::VertexId::Vector;

/// tactic types
using EnvInfo = vtr_tactic_msgs::msg::EnvInfo;
/** \brief the vertex creation test result */
enum class KeyframeTestResult : int {
  CREATE_VERTEX = 0,
  CREATE_CANDIDATE = 1,
  FAILURE = 2,
  DO_NOTHING = 3
};

}  // namespace tactic
}  // namespace vtr