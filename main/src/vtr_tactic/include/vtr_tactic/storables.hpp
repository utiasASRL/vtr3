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
 * \file storables.hpp
 * \brief Several storable classes for storing odometry and localization results
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_tactic/types.hpp"

#include "vtr_tactic_msgs/msg/localization_result.hpp"
#include "vtr_tactic_msgs/msg/odometry_result.hpp"

namespace vtr {
namespace tactic {

class OdometryResult {
 public:
  using OdometryResultMsg = vtr_tactic_msgs::msg::OdometryResult;

  static std::shared_ptr<OdometryResult> fromStorable(
      const OdometryResultMsg& storable);
  OdometryResultMsg toStorable() const;

  OdometryResult(const Timestamp& timestamp, const EdgeTransform& T_world_robot)
      : timestamp_{timestamp}, T_world_robot_{T_world_robot} {}

 private:
  /** \brief Timestamp of the frame to localize */
  Timestamp timestamp_;

  /** \brief Transformation from vertex local frame to robot live frame */
  EdgeTransform T_world_robot_;
};

class LocalizationResult {
 public:
  using LocalizationResultMsg = vtr_tactic_msgs::msg::LocalizationResult;

  static std::shared_ptr<LocalizationResult> fromStorable(
      const LocalizationResultMsg& storable);
  LocalizationResultMsg toStorable() const;

  LocalizationResult(const Timestamp& timestamp,
                     const Timestamp& vertex_timestamp,
                     const VertexId& vertex_id,
                     const EdgeTransform& T_robot_vertex)
      : timestamp_{timestamp},
        vertex_timestamp_{vertex_timestamp},
        vertex_id_{vertex_id},
        T_robot_vertex_{T_robot_vertex} {}

 private:
  /** \brief Timestamp of the frame to localize */
  Timestamp timestamp_;

  /** \brief Timestamp of the vertex being localized against */
  Timestamp vertex_timestamp_;

  /** \brief Vertex id being localized against */
  VertexId vertex_id_;

  /** \brief Transformation from vertex local frame to robot live frame */
  EdgeTransform T_robot_vertex_;
};

}  // namespace tactic
}  // namespace vtr