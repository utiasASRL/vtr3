#include <vtr/planning/event.h>
#include <vtr/planning/state_machine.h>

namespace vtr {
namespace planning {
namespace state {

std::ostream& operator<<(std::ostream& os, const Action& action) {
  switch (action) {
    case Action::Abort:
      os << "Abort";
      return os;
    case Action::Continue:
      os << "Continue";
      return os;
    case Action::NewGoal:
      os << "NewGoal";
      return os;
    case Action::SwapGoal:
      os << "SwapGoal";
      return os;
    case Action::AppendGoal:
      os << "AppendGoal";
      return os;
    case Action::EndGoal:
      os << "EndGoal";
      return os;
  };

  // GCC seems to complain when this isn't here
  return os;
}

std::ostream& operator<<(std::ostream& os, const Signal& signal) {
  switch (signal) {
    case Signal::Continue:
      os << "Continue";
      return os;
#if 0
    case Signal::CantLocalize:
      os << "CantLocalize";
      return os;
    case Signal::RobotLost:
      os << "RobotLost";
      return os;
    case Signal::VertexFound:
      os << "VertexFound";
      return os;
    case Signal::CantPlan:
      os << "CantPlan";
      return os;
    case Signal::PlanSuccess:
      os << "PlanSuccess";
      return os;
    case Signal::LocalizeObs:
      os << "LocalizationObstruction";
      return os;
    case Signal::Localized:
      os << "Localized";
      return os;
    case Signal::LocalizeFail:
      os << "LocalizationFailure";
      return os;
    case Signal::Obstructed:
      os << "Obstructed";
      return os;
    case Signal::GoalReached:
      os << "GoalReached";
      return os;
    case Signal::DoRepair:
      os << "DoRepair";
      return os;
    case Signal::AttemptClosure:
      os << "FindClosure";
      return os;
    case Signal::ContinueTeach:
      os << "ContinueTeach";
      return os;
#endif
  }

  // GCC seems to complain when this isn't here
  return os;
}

Event Event::StartIdle() {
  return Event(Action::NewGoal, typename BaseState::Ptr(new Idle()));
}

Event Event::StartTeach() {
  return Event(Action::NewGoal, typename BaseState::Ptr(new teach::Branch()));
}

#if 0
Event Event::StartMerge(const std::vector<VertexId>& matchWindow,
                        const VertexId& targetVertex) {
  typename state::teach::Merge::Ptr tmp(new teach::Merge());
  tmp->setTarget(matchWindow, targetVertex);
  return Event(Action::SwapGoal, tmp);
}

Event Event::StartMerge(const std::list<VertexId>& matchWindow,
                        const VertexId& targetVertex) {
  return StartMerge(
      std::vector<VertexId>(matchWindow.begin(), matchWindow.end()),
      targetVertex);
}

Event Event::StartLocalize(const std::vector<VertexId>& matchWindow,
                           const VertexId& targetVertex) {
  return StartLocalize(
      std::list<VertexId>(matchWindow.begin(), matchWindow.end()),
      targetVertex);
}

Event Event::StartLocalize(const std::list<VertexId>& matchWindow,
                           const VertexId& targetVertex) {
  typename state::repeat::TopologicalLocalize::Ptr tmp(
      new repeat::TopologicalLocalize());
  tmp->setWaypoints(matchWindow);
  tmp->setTarget(targetVertex);
  return Event(Action::NewGoal, tmp);
}

Event Event::StartRepeat(const std::list<VertexId>& waypoints) {
  typename state::repeat::Follow::Ptr tmp(new repeat::Follow());
  tmp->setWaypoints(waypoints);

  return Event(Action::NewGoal, tmp);
}
#endif
Event Event::Pause() {
  return Event(Action::AppendGoal, typename BaseState::Ptr(new Idle()));
}
#if 0
// UAV
Event Event::StartLearn() {
  return Event(Action::SwapGoal, typename BaseState::Ptr(new learn::Branch()));
}
Event Event::StartLoiter() {
  return Event(Action::SwapGoal,
               typename BaseState::Ptr(new loiter::TeachLoiter()));
}

Event Event::StartReturn(const std::list<VertexId>& waypoints) {
  typename state::rreturn::MetricLocalize::Ptr tmp(
      new rreturn::MetricLocalize());
  tmp->setWaypoints(waypoints);
  return Event(Action::SwapGoal, tmp);
}

Event Event::StartHover() {
  return Event(Action::SwapGoal,
               typename BaseState::Ptr(new hover::MetricLocalize()));
}

Event Event::StartJoin(const std::list<VertexId>& matchWindow,
                       const VertexId& targetVertex) {
  return StartJoin(
      std::vector<VertexId>(matchWindow.begin(), matchWindow.end()),
      targetVertex);
}

Event Event::StartJoin(const std::vector<VertexId>& matchWindow,
                       const VertexId& targetVertex) {
  typename state::learn::Merge::Ptr tmp(new learn::Merge());
  tmp->setTarget(matchWindow, targetVertex);
  return Event(Action::SwapGoal, tmp);
}
#endif

Event::Event(const Signal& signal)
    : type_(Action::Continue), goal_(nullptr), signal_(signal) {}

Event::Event(const Action& type, const StatePtr& goal)
    : type_(type), goal_(goal), signal_(Signal::Continue) {}

std::string Event::signalName() const {
  std::stringstream ss;
  ss << signal_;
  return ss.str();
}

std::string Event::actionName() const {
  std::stringstream ss;
  ss << type_;
  return ss.str();
}

std::ostream& operator<<(std::ostream& os, const Event& e) {
  if (e.goal_) {
    os << "<" << e.type_ << ": " << *e.goal_ << ", " << e.signal_ << ">";
  } else {
    os << "<" << e.type_ << ", " << e.signal_ << ">";
  }
  return os;
}

}  // namespace state
}  // namespace planning
}  // namespace vtr
