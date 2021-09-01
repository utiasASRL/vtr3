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
 * \file publisher_interface.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_tactic/types.hpp>

namespace vtr {
namespace tactic {

/** \brief Interface for a generic tactic state publisher */
class PublisherInterface {
 public:
  /** \brief Sets the path followed by the robot for UI update */
  virtual void publishPath(const LocalizationChain &chain) const = 0;
  /** \brief Clears the path followed by the robot for UI update */
  virtual void clearPath() const = 0;
  /** \brief Updates robot messages for UI */
  virtual void publishRobot(
      const Localization &persistentLoc, uint64_t pathSeq = 0,
      const Localization &targetLoc = Localization(),
      const std::shared_ptr<rclcpp::Time> stamp = nullptr) const = 0;
};

}  // namespace tactic
}  // namespace vtr
