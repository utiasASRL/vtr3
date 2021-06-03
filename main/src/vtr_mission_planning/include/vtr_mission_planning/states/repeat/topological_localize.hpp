#pragma once

#include <vtr_mission_planning/states/repeat.hpp>

namespace vtr {
namespace mission_planning {
namespace state {
namespace repeat {

class TopologicalLocalize : public Repeat {
 public:
  PTR_TYPEDEFS(TopologicalLocalize)
  INHERITANCE_TESTS(TopologicalLocalize, Base)
  using Parent = Repeat;

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
  /** \brief State through which we must always enter this meta-state */
  BasePtr entryState(const Base *) const override;
  /** \brief Checks the navigation state and perform state transitions */
  void processGoals(Tactic *tactic, UpgradableLockGuard &goal_lock,
                    const Event &event = Event()) override;
  /** \brief Called as a cleanup method when the state exits. */
  void onExit(Tactic *tactic, Base *newState) override;
  /** \brief Called as a setup method when the state is entered. */
  void onEntry(Tactic *tactic, Base *oldState) override;
};

}  // namespace repeat
}  // namespace state
}  // namespace mission_planning
}  // namespace vtr
