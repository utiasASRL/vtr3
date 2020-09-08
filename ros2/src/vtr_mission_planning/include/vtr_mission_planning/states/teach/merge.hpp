#pragma once

#include <vtr_mission_planning/states/teach.hpp>

namespace vtr {
namespace mission_planning {
namespace state {
#if 0
class Teach;
class Event;

enum class Signal : int8_t;
enum class Action : int8_t;
#endif

namespace teach {

class Merge : public Teach {
 public:
  PTR_TYPEDEFS(Merge)
  INHERITANCE_TESTS(Merge, Base)
  using Parent = Teach;
#if 0
  using Base = Parent::Base;
  using BasePtr = Base::Ptr;
  using Tactic = Parent::Tactic;

  using VertexId = asrl::pose_graph::VertexId;
#endif

  Merge(const Parent &parent = Parent()) : Parent(parent), cancelled_(false) {}
  Merge(const Base &base) : Parent(base), cancelled_(false) {}
  Merge(const Merge &) = default;
  Merge(Merge &&) = default;

  virtual ~Merge() {}

  Merge &operator=(const Merge &) = default;
  Merge &operator=(Merge &&) = default;

  /** \brief Gets an enum representing the type of pipeline that this state
   * requires
   */
  virtual PipelineType pipeline() const { return PipelineType::Merge; }

  /** \brief Return a string representation of the state
   */
  virtual std::string name() const { return Parent::name() + "::Merge"; };

  /** \brief Get the next intermediate state, for when no direct transition is
   * possible
   */
  virtual BasePtr nextStep(const Base *newState) const;

  /** \brief The entryState function is not implemented for leaf states
   */

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
  bool cancelled_;

  bool canCloseLoop_(Tactic *tactic);
};

}  // namespace teach
}  // namespace state
}  // namespace mission_planning
}  // namespace vtr
