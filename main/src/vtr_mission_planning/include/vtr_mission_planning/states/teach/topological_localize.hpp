#pragma once

#include <vtr_mission_planning/states/teach.hpp>

namespace vtr {
namespace mission_planning {
namespace state {

namespace teach {

class TopologicalLocalize : public Teach {
 public:
  PTR_TYPEDEFS(TopologicalLocalize)
  INHERITANCE_TESTS(TopologicalLocalize, Base)
  using Parent = Teach;

  TopologicalLocalize(const Parent &parent = Parent()) : Parent(parent) {}
  TopologicalLocalize(const Base &base) : Parent(base) {}
  TopologicalLocalize(const TopologicalLocalize &) = default;
  TopologicalLocalize(TopologicalLocalize &&) = default;

  virtual ~TopologicalLocalize() {}

  TopologicalLocalize &operator=(const TopologicalLocalize &) = default;
  TopologicalLocalize &operator=(TopologicalLocalize &&) = default;

  /** \brief Return a string representation of the state */
  std::string name() const override {
    return Parent::name() + "::TopologicalLocalize";
  }
  /** \brief Returns the type of pipeline that this state requires. */
  PipelineMode pipeline() const override { return PipelineMode::Idle; }
  /** \brief Returns the next intermediate state */
  BasePtr nextStep(const Base *newState) const override;
  /** \brief The entryState function is not implemented for leaf states */

  /** \brief Checks the navigation state and perform state transitions */
  void processGoals(Tactic *tactic, UpgradableLockGuard &goal_lock,
                    const Event &event = Event()) override;
  /** \brief Called as a cleanup method when the state exits. */
  void onExit(Tactic *tactic, Base *newState) override;
  /** \brief Called as a setup method when the state is entered. */
  void onEntry(Tactic *tactic, Base *oldState) override;
};

}  // namespace teach
}  // namespace state
}  // namespace mission_planning
}  // namespace vtr
