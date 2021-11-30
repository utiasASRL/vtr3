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
 * \file state_machine_interface.hpp
 * \brief Interface for state machines
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_common/utils/macros.hpp>
#include <vtr_tactic/types.hpp>

namespace vtr {
namespace tactic {

/** \brief Defines the possible pipeline types to be used by tactics */
enum class PipelineMode : uint8_t {
  Idle,       // Idle
  Branching,  // Teach - branching from existing path
  Merging,    // Teach - merging into existing path
  Following,  // Repeat - path following
  Searching,  //
};

/** \brief Possible localization statuses */
enum class LocalizationStatus : uint8_t {
  Confident,      // If we're trying to localize, we did. If not, VO is happy.
  Forced,         // If we have forcibly set the localization, but not actually
                  // localized
  DeadReckoning,  // The last frame did not localize against the map, but we are
                  // within allowable VO range
  LOST  // We have been on VO for too long or VO has failed, and need to stop
};

/** \brief Possible status returns from the safety module */
enum class SafetyStatus : uint8_t {
  // TODO IMPORTANT: These should always be in increasing order of severity, and
  // should never be given numbers
  Safe,         // Nothing is wrong; keep going
  NotThatSafe,  // Something is questionable and we should maybe slow down
  DANGER        // STAHP! What are you doing??
};

/** \brief Combined status message */
struct TacticStatus {
  LocalizationStatus localization_;
  LocalizationStatus targetLocalization_;
  SafetyStatus safety_;
};

/** \brief Full metric and topological localization in one package */
struct Localization {
  Localization(const VertexId& vertex = VertexId::Invalid(),
               const EdgeTransform& T_robot_vertex = EdgeTransform(),
               bool hasLocalized = false, int numSuccess = 0)
      : v(vertex),
        T(T_robot_vertex),
        localized(hasLocalized),
        successes(numSuccess) {
    // Initialize to a reasonably large covariance if no transform is specified
    if (!T.covarianceSet()) {
      T.setCovariance(Eigen::Matrix<double, 6, 6>::Identity());
    }
  }
  storage::Timestamp stamp = -1;
  VertexId v;
  EdgeTransform T;
  bool localized;
  int successes;
};

/**
 * \brief Interface that a tactic must implement to be compatible with the
 * state machine
 */
class StateMachineInterface {
 public:
  PTR_TYPEDEFS(StateMachineInterface)

  using LockType = std::unique_lock<std::recursive_timed_mutex>;

  virtual ~StateMachineInterface() = default;

  /**
   * \brief Clears the pipeline and stops callbacks.
   * \returns a lock that blocks the pipeline
   */
  virtual LockType lockPipeline() { return LockType(); }
  /** \brief Set the pipeline used by the tactic */
  virtual void setPipeline(const PipelineMode& pipeline) = 0;
  /** \brief Set the path being followed */
  virtual void setPath(const PathType& path, bool follow = false) = 0;
  /** \brief Set the current privileged vertex (topological localization) */
  virtual void setTrunk(const VertexId& v) = 0;
  /** \brief Get distance between the current loc. chain to the target vertex */
  virtual double distanceToSeqId(const uint64_t& idx) = 0;
  /** \brief Returns whether the path following has completed */
  /** \brief Add a new run to the graph and reset localization flags */
  virtual void addRun(bool ephemeral = false) = 0;
#if 0
  /** \brief Remove any temporary runs */
  virtual void removeEphemeralRuns() = 0;
#endif
  virtual bool pathFollowingDone() = 0;
  /** \brief Whether or not can merge into existing graph. */
  virtual bool canCloseLoop() const = 0;
  /** \brief Add a new vertex, link it to the current trunk and branch */
  virtual void connectToTrunk(bool privileged = false, bool merge = false) = 0;
  /** \brief Get the current localization and safety status */
  virtual TacticStatus status() const = 0;
  /** \brief Get how confident we are in the localization */
  virtual LocalizationStatus tfStatus(const EdgeTransform& tf) const = 0;
  /** \brief Get the persistent localization */
  virtual const Localization& persistentLoc() const = 0;
  /** \brief Get the target localization */
  virtual const Localization& targetLoc() const = 0;
  /** \brief Get the current vertex ID */
  virtual const VertexId& currentVertexID() const = 0;
  /** \brief Get the closest vertex to the current position */
  virtual const VertexId& closestVertexID() const = 0;
  /** \brief Trigger a graph relaxation */
  virtual void relaxGraph() = 0;
  /** \brief Save the graph */
  virtual void saveGraph() {}
};

}  // namespace tactic
}  // namespace vtr