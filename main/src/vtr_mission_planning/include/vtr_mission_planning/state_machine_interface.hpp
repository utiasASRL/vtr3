#pragma once

#include <lgmath.hpp>
#include <vtr_common/utils/macros.hpp>
#include <vtr_pose_graph/id/graph_id.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>

namespace vtr {
namespace mission_planning {

using Transform = lgmath::se3::TransformationWithCovariance;
using PathType = vtr::pose_graph::VertexId::Vector;
using VertexId = vtr::pose_graph::VertexId;

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
               const Transform& T_robot_vertex = Transform(),
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

  VertexId v;
  Transform T;
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
  using VertexId = vtr::pose_graph::VertexId;
  using EdgeId = vtr::pose_graph::EdgeId;
  using Graph = vtr::pose_graph::RCGraph;

  using LockType = std::unique_lock<std::recursive_timed_mutex>;

  virtual ~StateMachineInterface() = default;

  /** \brief Set the pipeline used by the tactic */
  virtual void setPipeline(const PipelineMode& pipeline) = 0;

  /**
   * \brief Clears the pipeline and stops callbacks.
   * \returns a lock that blocks the pipeline
   */
  virtual LockType lockPipeline() { return LockType(); }
  /** \brief Set the path being followed */
  virtual void setPath(const PathType& path, bool follow = false) = 0;
  /** \brief Set the current privileged vertex (topological localization) */
  virtual void setTrunk(const VertexId& v) = 0;
  /**
   * \brief Get the distance along the current localization chain to the target
   * vertex
   */
  virtual double distanceToSeqId(const uint64_t& idx) = 0;
  /** \brief Get the current localization and safety status */
  virtual TacticStatus status() const = 0;

  /** \brief Get how confident we are in the localization */
  virtual LocalizationStatus tfStatus(
      const pose_graph::RCEdge::TransformType& tf) const = 0;

  /** \brief Whether or not can merge into existing graph. */
  virtual bool canCloseLoop() const = 0;

  /**
   * \brief Add a new vertex if necessary and link it to the current trunk and
   * branch vertices
   */
  virtual void connectToTrunk(bool privileged = false) = 0;
  /** \brief Get the persistent localization */
  virtual const Localization& persistentLoc() const = 0;
  /** \brief Get the target localization */
  virtual const Localization& targetLoc() const = 0;

  /** \brief Get the current vertex ID */
  virtual const VertexId& currentVertexID() const = 0;

  /** \brief Get the closest vertex to the current position */
  virtual const VertexId& closestVertexID() const = 0;

  /** \brief Update the localization success count */
  virtual void incrementLocCount(int8_t) {}

  /** \brief Add a new run to the graph and reset localization flags */
  virtual void addRun(bool ephemeral = false, bool extend = false,
                      bool save = true) = 0;
#if 0
  /** \brief Remove any temporary runs */
  virtual void removeEphemeralRuns() = 0;
#endif
  /** \brief Trigger a graph relaxation */
  virtual void relaxGraph() = 0;

  /** \brief Save the graph */
  virtual void saveGraph() {}

  /** \brief Get a copy of the pose graph */
  virtual std::shared_ptr<Graph> poseGraph() { return nullptr; }
};

namespace state {
class BaseState;
}

class StateMachineCallbacks {
 public:
  PTR_TYPEDEFS(StateMachineCallbacks)
  virtual void stateAbort(const std::string&) = 0;
  virtual void stateChanged(const std::shared_ptr<state::BaseState>&) = 0;
  virtual void stateSuccess() = 0;
  virtual void stateUpdate(double) = 0;
};

}  // namespace mission_planning
}  // namespace vtr