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
 * \file edge_id.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once
#if false
#include "vtr_pose_graph/id/base_id.hpp"

namespace vtr {
namespace pose_graph {

enum class EdgeIdType { Temporal = 0, Spatial, UNDEFINED };

class EdgeId : public BasePairedId<EdgeIdType> {
 public:
  CONTAINER_TYPEDEFS(EdgeId);
  using ListArray = std::array<List, EdgeId::NumTypes()>;
  using SetArray = std::array<Set, EdgeId::NumTypes()>;
  using VectorArray = std::array<Vector, EdgeId::NumTypes()>;
  using UnorderedSetArray = std::array<UnorderedSet, EdgeId::NumTypes()>;

  using Type = EdgeIdType;

  static constexpr EdgeId Invalid() { return EdgeId(); }

  constexpr EdgeId() = default;
  constexpr EdgeId(const CombinedIdType &id1, const CombinedIdType &id2,
                   const EdgeIdType &type)
      : BasePairedId(id1, id2, type) {}
};
#endif
}  // namespace pose_graph
}  // namespace vtr

EXTEND_HASH(vtr::pose_graph::EdgeId);