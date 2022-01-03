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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 * \brief Several storable classes for storing odometry and localization results
 */

#include "vtr_tactic/storables.hpp"

#include "vtr_common/conversions/ros_lgmath.hpp"

namespace vtr {
namespace tactic {

std::shared_ptr<OdometryResult> OdometryResult::fromStorable(
    const OdometryResultMsg& storable) {
  const auto timestamp = storable.timestamp;
  EdgeTransform T_world_robot;
  common::conversions::fromROSMsg(storable.t_world_robot, T_world_robot);

  auto data = std::make_shared<OdometryResult>(timestamp, T_world_robot);
  return data;
}

auto OdometryResult::toStorable() const -> OdometryResultMsg {
  OdometryResultMsg storable;
  storable.timestamp = timestamp_;
  common::conversions::toROSMsg(T_world_robot_, storable.t_world_robot);
  return storable;
}

std::shared_ptr<LocalizationResult> LocalizationResult::fromStorable(
    const LocalizationResultMsg& storable) {
  const auto timestamp = storable.timestamp;
  const auto vertex_timestamp = storable.vertex_timestamp;
  const auto vertex_id = storable.vertex_id;
  EdgeTransform T_robot_vertex;
  common::conversions::fromROSMsg(storable.t_robot_vertex, T_robot_vertex);

  auto data = std::make_shared<LocalizationResult>(timestamp, vertex_timestamp,
                                                   vertex_id, T_robot_vertex);
  return data;
}

auto LocalizationResult::toStorable() const -> LocalizationResultMsg {
  LocalizationResultMsg storable;
  storable.timestamp = timestamp_;
  storable.vertex_timestamp = vertex_timestamp_;
  storable.vertex_id = vertex_id_;
  common::conversions::toROSMsg(T_robot_vertex_, storable.t_robot_vertex);
  return storable;
}

}  // namespace tactic
}  // namespace vtr