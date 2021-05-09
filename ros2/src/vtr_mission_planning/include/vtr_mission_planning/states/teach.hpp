#pragma once

#include <vtr_mission_planning/state_machine.hpp>

namespace vtr {
namespace mission_planning {
namespace state {

class Teach : public BaseState {
 public:
  PTR_TYPEDEFS(Teach)
  INHERITANCE_TESTS(Teach, Base)
  using Parent = BaseState;

  Teach(const Base &base = Base()) : Parent(base) {}
  Teach(const Teach &) = default;
  Teach(Teach &&) = default;

  virtual ~Teach() {}

  Teach &operator=(const Teach &) = default;
  Teach &operator=(Teach &&) = default;

  /** \brief Return a string representation of the state */
  std::string name() const override { return Parent::name() + "::Teach"; }
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

  /** \brief Set the target to match against */
  void setTarget(const std::vector<VertexId> &matchWindow,
                 const VertexId &targetVertex) {
    matchWindow_ = matchWindow;
    targetVertex_ = targetVertex;
  }

 protected:
  /** \brief Window of vertices to search against for a match/localization */
  std::vector<VertexId> matchWindow_;

  /** \brief Target vertex to rejoin to/start mapping from */
  VertexId targetVertex_;
};
}  // namespace state
}  // namespace mission_planning
}  // namespace vtr

// We chain includes here for convenience
#include <vtr_mission_planning/states/teach/branch.hpp>
#include <vtr_mission_planning/states/teach/merge.hpp>
#include <vtr_mission_planning/states/teach/topological_localize.hpp>
