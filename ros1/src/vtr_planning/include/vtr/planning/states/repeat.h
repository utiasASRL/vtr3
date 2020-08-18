#pragma once

#include <vtr/planning/state_machine.h>

#include <asrl/common/utils/CommonMacros.hpp>
#include <asrl/pose_graph/id/GraphId.hpp>

namespace vtr {
namespace planning {
namespace state {

#if 0
class BaseState;
class Event;
#endif

class Repeat : public BaseState {
 public:
  PTR_TYPEDEFS(Repeat)
  DEFAULT_COPY_MOVE(Repeat)
  INHERITANCE_TESTS(Repeat, Base)
  using Parent = BaseState;
#if 0  
  using Base = Parent::Base;
  using BasePtr = Base::Ptr;
  using Tactic = Parent::Tactic;
#endif
  using VertexId = asrl::pose_graph::VertexId;

  Repeat(const Base &base = Base())
      : Parent(base), targetVertex_(VertexId::Invalid()) {}
  virtual ~Repeat() {}

  /** \brief Return a string representation of the state
   */
  virtual std::string name() const { return Parent::name() + "::Repeat"; }
  /** \brief Set the list of waypoints to follow
   */
  void setWaypoints(
      const std::list<VertexId> &waypoints,
      const std::list<uint64_t> &waypointSeq = std::list<uint64_t>()) {
    waypoints_ = waypoints;
    waypointSeq_ = waypointSeq;
  }

  /** \brief Set the target vertex for localization
   */
  void setTarget(const VertexId &target) { targetVertex_ = target; }

  /** \brief Get the next intermediate state, for when no direct transition is
   * possible
   */
  virtual BasePtr nextStep(const Base *newState) const;

  /** \brief State through which we must always enter this meta-state
   */
  virtual BasePtr entryState(const Base *) const;

  /** \brief Check the navigation state and perform necessary state transitions
   */
  virtual void processGoals(Tactic *tactic, UpgradableLockGuard &goal_lock,
                            const Event &event = Event());

  /** \brief Called as a cleanup method when the state exits.  The base state
   * never exits.
   */
  virtual void onExit(Tactic *tactic, Base *newState);

  /** \brief Called as a setup method when the state is entered.  The base state
   * is never entered explicitly.
   */
  virtual void onEntry(Tactic *tactic, Base *oldState);

 protected:
  /** \brief Vector of vertex ids that the robot is trying to get to as it's
   * objective
   */
  std::list<VertexId> waypoints_;

  /** \brief Vertex of sequence ids of the waypoints along the current path
   */
  std::list<uint64_t> waypointSeq_;

  /** \brief Target vertex to localize against
   */
  VertexId targetVertex_;
};

}  // namespace state
}  // namespace planning
}  // namespace vtr

// We chain the includes here for convenience
#include <vtr/planning/states/repeat/follow.h>
#include <vtr/planning/states/repeat/metric_localize.h>
#include <vtr/planning/states/repeat/plan.h>
#include <vtr/planning/states/repeat/topological_localize.h>
