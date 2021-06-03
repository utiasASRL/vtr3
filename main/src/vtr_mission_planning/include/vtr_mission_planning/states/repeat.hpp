#pragma once

#include <vtr_mission_planning/state_machine.hpp>
#include <vtr_pose_graph/id/graph_id.hpp>

namespace vtr {
namespace mission_planning {
namespace state {

class Repeat : public BaseState {
 public:
  PTR_TYPEDEFS(Repeat)
  INHERITANCE_TESTS(Repeat, Base)
  using Parent = BaseState;

  Repeat(const Base &base = Base())
      : Parent(base), targetVertex_(VertexId::Invalid()) {}
  Repeat(const Repeat &) = default;
  Repeat(Repeat &&) = default;

  virtual ~Repeat() {}

  Repeat &operator=(const Repeat &) = default;
  Repeat &operator=(Repeat &&) = default;

  /** \brief Return a string representation of the state */
  std::string name() const override { return Parent::name() + "::Repeat"; }
  /** \brief Returns the next intermediate state */
  BasePtr nextStep(const Base *newState) const override;
  /** \brief State through which we must always enter this meta-state */
  BasePtr entryState(const Base *) const override;
  /** \brief Checks the navigation state and perform state transitions */
  void processGoals(Tactic *tactic, UpgradableLockGuard &goal_lock,
                    const Event &event = Event()) override;
  /** \brief Called as a cleanup method when the state exits. */
  void onExit(Tactic *tactic, Base *newState) override;
  /** \brief Called as a setup method when the state is entered. */
  void onEntry(Tactic *tactic, Base *oldState) override;

  /** \brief Set the list of waypoints to follow */
  void setWaypoints(
      const std::list<VertexId> &waypoints,
      const std::list<uint64_t> &waypointSeq = std::list<uint64_t>()) {
    waypoints_ = waypoints;
    waypointSeq_ = waypointSeq;
  }

  /** \brief Set the target vertex for localization */
  void setTarget(const VertexId &target) { targetVertex_ = target; }

 protected:
  /**
   * \brief Vector of vertex ids that the robot is trying to get to as it's
   * objective
   */
  std::list<VertexId> waypoints_;

  /** \brief Vertex of sequence ids of the waypoints along the current path */
  std::list<uint64_t> waypointSeq_;

  /** \brief Target vertex to localize against */
  VertexId targetVertex_;
};

}  // namespace state
}  // namespace mission_planning
}  // namespace vtr

// We chain the includes here for convenience
#include <vtr_mission_planning/states/repeat/follow.hpp>
#include <vtr_mission_planning/states/repeat/metric_localize.hpp>
#include <vtr_mission_planning/states/repeat/plan.hpp>
#include <vtr_mission_planning/states/repeat/topological_localize.hpp>
