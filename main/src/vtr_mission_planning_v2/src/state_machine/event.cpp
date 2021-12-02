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
 * \file event.cpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_mission_planning_v2/state_machine/event.hpp"
#include "vtr_mission_planning_v2/state_machine/state_interface.hpp"

namespace vtr {
namespace mission_planning {

std::ostream& operator<<(std::ostream& os, const Action& action) {
  switch (action) {
    case Action::Abort:
      os << "Abort";
      return os;
    case Action::Reset:
      os << "Reset";
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
    case Signal::LocalizeFail:
      os << "LocalizationFailure";
      return os;
    case Signal::GoalReached:
      os << "GoalReached";
      return os;
    case Signal::AttemptClosure:
      os << "AttemptClosure";
      return os;
    case Signal::SwitchMergeWindow:
      os << "SwitchMergeWindow";
      return os;
    case Signal::ContinueTeach:
      os << "ContinueTeach";
      return os;
  }

  // GCC seems to complain when this isn't here
  return os;
}

#if false
Event Event::StartIdle() {
  return Event(Action::NewGoal, typename BaseState::Ptr(new Idle()));
}

Event Event::StartTeach() {
  return Event(Action::NewGoal, typename BaseState::Ptr(new teach::Branch()));
}

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

Event Event::Reset() {
  return Event(Action::Reset, typename BaseState::Ptr(new Idle()));
}
#endif

std::string Event::signalName() const {
  std::stringstream ss;
  ss << signal;
  return ss.str();
}

std::string Event::actionName() const {
  std::stringstream ss;
  ss << action;
  return ss.str();
}

std::ostream& operator<<(std::ostream& os, const Event& e) {
  if (e.goal)
    os << "<" << e.action << ": " << *e.goal << ", " << e.signal << ">";
  else
    os << "<" << e.action << ", " << e.signal << ">";
  return os;
}

}  // namespace mission_planning
}  // namespace vtr
