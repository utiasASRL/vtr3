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
 * \file tactic_callback_interface.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_tactic/types.hpp"

namespace vtr {
namespace tactic {

class TacticCallbackInterface {
 public:
  using Ptr = std::shared_ptr<TacticCallbackInterface>;

  virtual ~TacticCallbackInterface() = default;

  /** \brief callback when a run is about to start (as entering teach/repeat) */
  virtual void startRun() {}
  /** \brief callback when a run is about to finish (as exiting teach/repeat) */
  virtual void endRun() {}
  /** \brief callback on robot state updated: persistent, target */
  virtual void robotStateUpdated(const Localization& /* persistent_loc */,
                                 const Localization& /* target_loc */) {}
  /** \brief callback on following path updated */
  virtual void pathUpdated(const VertexId::Vector& /* path */) {}

  virtual void publishOdometryRviz(const Timestamp& /* stamp */,
                                   const std::string& /* robot_frame */,
                                   const EdgeTransform& /* T_r_m_odo */,
                                   const EdgeTransform& /* T_w_m_odo */) {}
  virtual void publishPathRviz(const LocalizationChain& /* chain */) {}
  virtual void publishLocalizationRviz(const Timestamp& /* stamp */,
                                       const EdgeTransform& /* T_w_m_loc */) {}
};

}  // namespace tactic
}  // namespace vtr