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
 * \file tactic_interface.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_common/utils/macros.hpp"
#include "vtr_tactic/types.hpp"

namespace vtr {
namespace tactic {

class TacticInterface {
 public:
  PTR_TYPEDEFS(TacticInterface);

  using Mutex = std::recursive_timed_mutex;
  using UniqueLock = std::unique_lock<Mutex>;

  virtual ~TacticInterface() = default;

  /**
   * \brief Clears the pipeline and stops callbacks.
   * \returns a lock that blocks the pipeline
   */
  virtual UniqueLock lockPipeline() = 0;
  /** \brief Set the pipeline used by the tactic */
  virtual void setPipeline(const PipelineMode& pipeline) = 0;
  /** \brief Set the path being followed */
  virtual void setPath(const PathType& path, bool follow = false) = 0;
  /** \brief Set the current privileged vertex (topological localization) */
  virtual void setTrunk(const VertexId& v) = 0;
  /** \brief Get distance between the current loc. chain to the target vertex */
  virtual double distanceToSeqId(const uint64_t& idx) = 0;
  /** \brief Add a new run to the graph and reset localization flags */
  virtual void addRun(bool ephemeral = false) = 0;
  /** \brief Indicate that the current run has finished */
  virtual void finishRun() = 0;
  /** \brief  */
  virtual bool pathFollowingDone() = 0;
  /** \brief Whether or not can merge into existing graph. */
  virtual bool canCloseLoop() const = 0;
  /** \brief Add a new vertex, link it to the current trunk and branch */
  virtual void connectToTrunk(bool privileged = false, bool merge = false) = 0;
  virtual const Localization& persistentLoc() const = 0;
};

}  // namespace tactic
}  // namespace vtr