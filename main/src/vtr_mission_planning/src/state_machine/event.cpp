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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_mission_planning/state_machine/event.hpp"
#include "vtr_mission_planning/state_machine/base_state.hpp"

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
    case Action::ForceAddVertex:
      os << "ForceAddVertex";
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
    case Signal::AttemptClosure:
      os << "AttemptClosure";
      return os;
    case Signal::ContinueTeach:
      os << "ContinueTeach";
      return os;
  }

  // GCC seems to complain when this isn't here
  return os;
}

Event::Ptr Event::StartIdle(const VertexId& v) {
  const auto tmp = std::make_shared<Idle>();
  tmp->setVertexId(v);
  return std::make_shared<Event>(Action::NewGoal, tmp);
}

Event::Ptr Event::StartTeach() {
  return std::make_shared<Event>(Action::NewGoal,
                                 std::make_shared<teach::Branch>());
}

Event::Ptr Event::StartMerge(const std::vector<VertexId>& match_window) {
  const auto tmp = std::make_shared<teach::Merge>();
  tmp->setTarget(match_window);
  return std::make_shared<Event>(Action::SwapGoal, tmp);
}

Event::Ptr Event::StartRepeat(const std::list<VertexId>& waypoints) {
  auto tmp = std::make_shared<repeat::Follow>();
  tmp->setWaypoints(waypoints);
  return std::make_shared<Event>(Action::NewGoal, tmp);
}

Event::Ptr Event::StartLocalize() {
  auto tmp = std::make_shared<localize::MetricLocalize>();
  return std::make_shared<Event>(Action::NewGoal, tmp);
}

Event::Ptr Event::Reset() { return std::make_shared<Event>(Action::Reset); }

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
