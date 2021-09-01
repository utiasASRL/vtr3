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
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <memory>

#include <vtr_mission_planning/state_machine_interface.hpp>
#include <vtr_pose_graph/evaluator/common.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#include <vtr_pose_graph/path/localization_chain.hpp>

namespace vtr {
namespace tactic {

/// pose graph structures
using Graph = pose_graph::RCGraph;
using GraphBase = pose_graph::RCGraphBase;
using RunId = pose_graph::RCRun::IdType;
using RunIdSet = std::set<RunId>;
using Vertex = pose_graph::RCVertex;
using VertexId = pose_graph::VertexId;
using EdgeId = pose_graph::EdgeId;
using EdgeTransform = pose_graph::RCEdge::TransformType;
/**
 * \brief Privileged edge mask.
 * \details This is used to create a subgraph on privileged edges.
 */
using PrivilegedEvaluator = pose_graph::eval::Mask::PrivilegedDirect<Graph>;
/** \brief Temporal edge mask. */
using TemporalEvaluator = pose_graph::eval::Mask::SimpleTemporalDirect<Graph>;
using LocalizationChain = pose_graph::LocalizationChain;

/// mission planning
using PipelineMode = mission_planning::PipelineMode;
using Localization = mission_planning::Localization;

/** \brief the vertex creation test result */
enum class KeyframeTestResult : int {
  CREATE_VERTEX = 0,
  CREATE_CANDIDATE = 1,
  FAILURE = 2,
  DO_NOTHING = 3
};
}  // namespace tactic
}  // namespace vtr