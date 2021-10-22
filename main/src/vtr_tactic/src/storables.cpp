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
 * \file storables.cpp
 * \brief Several storable classes for storing odometry and localization results
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_tactic/storables.hpp"

#include "vtr_common/lgmath/conversions.hpp"

namespace vtr {
namespace tactic {

std::shared_ptr<LocalizationResult> LocalizationResult::fromStorable(
    const LocalizationResultMsg& storable) {
  const auto timestamp = storable.timestamp;
  const auto vertex_timestamp = storable.vertex_timestamp;
  const auto vertex_id = storable.vertex_id;
  TransformType T_robot_vertex;
  common::fromROSMsg(storable.t_robot_vertex, T_robot_vertex);

  auto data = std::make_shared<LocalizationResult>(timestamp, vertex_timestamp,
                                                   vertex_id, T_robot_vertex);
  return data;
}

auto LocalizationResult::toStorable() const -> LocalizationResultMsg {
  LocalizationResultMsg storable;
  storable.timestamp = timestamp_;
  storable.vertex_timestamp = vertex_timestamp_;
  storable.vertex_id = vertex_id_;
  common::toROSMsg(T_robot_vertex_, storable.t_robot_vertex);
  return storable;
}

}  // namespace tactic
}  // namespace vtr