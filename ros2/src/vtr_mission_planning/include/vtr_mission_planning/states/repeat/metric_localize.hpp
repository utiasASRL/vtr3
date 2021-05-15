#pragma once

#include <vtr_mission_planning/states/repeat.hpp>

namespace vtr {
namespace mission_planning {
namespace state {

namespace repeat {

class MetricLocalize : public Repeat {
 public:
  PTR_TYPEDEFS(MetricLocalize)
  INHERITANCE_TESTS(MetricLocalize, Base)
  using Parent = Repeat;

  MetricLocalize(const Parent &parent = Parent()) : Parent(parent) {}
  MetricLocalize(const Base &base) : Parent(base) {}
  MetricLocalize(const MetricLocalize &) = default;
  MetricLocalize(MetricLocalize &&) = default;

  virtual ~MetricLocalize() {}

  MetricLocalize &operator=(const MetricLocalize &) = default;
  MetricLocalize &operator=(MetricLocalize &&) = default;

  /** \brief Return a string representation of the state */
  std::string name() const override {
    return Parent::name() + "::MetricLocalize";
  }
  /** \brief Returns the type of pipeline that this state requires. */
  PipelineMode pipeline() const override { return PipelineMode::Searching; }
  /** \brief Returns the next intermediate state */
  BasePtr nextStep(const Base *newState) const override;
  /** \brief State through which we must always enter this meta-state */

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
