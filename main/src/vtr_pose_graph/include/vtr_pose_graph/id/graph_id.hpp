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
 * \file graph_id.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/id/id_base.hpp>

namespace vtr {
namespace pose_graph {

DEFINE_ID(VertexId);
DEFINE_PAIRED_ID(EdgeId, Temporal, Spatial);

}  // namespace pose_graph
}  // namespace vtr

// This needs to happen outside the vtr::pose_graph namespace
EXTEND_HASH(vtr::pose_graph::VertexId);
EXTEND_HASH(vtr::pose_graph::EdgeId);