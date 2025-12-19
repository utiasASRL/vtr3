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
 * \file plan.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_mission_planning/state_machine/states/repeat/plan.hpp"
#include "vtr_route_planning/bfs_planner.hpp"
#include "vtr_route_planning/tdsp_planner.hpp"

#include <sstream>

namespace vtr {
namespace mission_planning {
namespace repeat {

StateInterface::Ptr Plan::nextStep(const StateInterface &new_state) const {
  // If where we are going is not a child, delegate to the parent
  if (!InChain(new_state)) return Parent::nextStep(new_state);

  // If we aren't changing to a different chain, there is no intermediate step
  return nullptr;
}

void Plan::processGoals(StateMachine &state_machine, const Event &event) {
  switch (event.signal) {
    case Signal::Continue:
      break;
    default:
      return Parent::processGoals(state_machine, event);
  }

  switch (event.action) {
    case Action::Continue:
      if (!getTactic(state_machine)->getPersistentLoc().v.isValid()) {
        // If we are lost, re-do topological localization
        const auto tmp = std::make_shared<TopologicalLocalize>();
        tmp->setWaypoints(waypoints_, waypoint_seq_);
        return Parent::processGoals(state_machine,
                                    Event(Action::AppendGoal, tmp));
      } else {
        // If we are localized, then continue on to repeat
        return Parent::processGoals(state_machine, Event(Action::EndGoal));
      }
    default:
      return Parent::processGoals(state_machine, event);
  }
}

void Plan::onExit(StateMachine &state_machine, StateInterface &new_state) {
  // If the new target is a derived class, we are not exiting
  if (InChain(new_state) && !IsType(new_state)) return;

  // Note: This is called *before* we call up the tree, as we destruct from
  // leaves to root
  if (MetricLocalize::InChain(new_state)) {
    const auto tactic = getTactic(state_machine);
    const auto persistent_loc = tactic->getPersistentLoc();
    const auto route_planner = getRoutePlanner(state_machine);
    const auto path_planner = getPathPlanner(state_machine);

    CLOG(INFO, "mission.state_machine")
        << "Current vertex: " << persistent_loc.v
        << ", waypoints: " << waypoints_;
    CLOG(INFO, "mission.state_machine")
        << "HSHMAT: Plan::onExit - Starting route computation (planner-agnostic)";

    PathType path;
        try {
          path = route_planner->path(persistent_loc.v, waypoints_, waypoint_seq_);
    } catch (const std::exception &e) {
          CLOG(ERROR, "mission.state_machine")
          << "HSHMAT: Route planning failed: '" << e.what()
              << "'. Keeping previous path; skipping setPath.";
          if (auto cb = getCallback(state_machine))
            cb->notifyRerouteStatus("reroute_failed");
          Parent::onExit(state_machine, new_state);
          return;
        }

    // ---------------------------------------------------------------------
    // Logging: planned path + ETA / cost breakdown (mission.state_machine).
    // This restores the "usual" debugging info from the previous implementation,
    // while keeping the state machine planner-agnostic.
    // ---------------------------------------------------------------------
    auto format_path_preview = [](const PathType &p, const size_t n = 6) -> std::string {
      std::stringstream ss;
      if (p.empty()) return "<empty>";
      if (p.size() <= 2 * n) {
        for (const auto &v : p) ss << v << " ";
        return ss.str();
      }
      for (size_t i = 0; i < n; ++i) ss << p[i] << " ";
      ss << "... ";
      for (size_t i = p.size() - n; i < p.size(); ++i) ss << p[i] << " ";
      return ss.str();
    };

    CLOG(INFO, "mission.state_machine")
        << "HSHMAT: Planned path preview: " << format_path_preview(path)
        << " (vertices=" << path.size() << ")";

    // If planner is BFSPlanner (Dijkstra/timecost), reuse its existing debug helper.
    if (auto bfs = std::dynamic_pointer_cast<vtr::route_planning::BFSPlanner>(route_planner)) {
      if (!path.empty()) {
        double delay_sec = 0.0;
        const double total_cost = bfs->debugPathTotalCost(path, delay_sec);
        CLOG(INFO, "mission.state_machine")
            << "HSHMAT: Planned path time-to-goal (Dijkstra/timecost): total="
            << total_cost << "s (extra_delay=" << delay_sec
            << "s, nominal_speed=" << bfs->nominalSpeed()
            << " m/s, use_time_cost=" << (bfs->useTimeCost() ? "true" : "false")
            << ")";
      }
    }

    // If planner is TDSP, compute ETA using the same FIFO arc-arrival model.
    if (auto tdsp = std::dynamic_pointer_cast<vtr::route_planning::TDSPPlanner>(route_planner)) {
      if (!path.empty()) {
        const double depart_sec = tdsp->nowSec();
        double wait_sec = 0.0, travel_sec = 0.0, static_sec = 0.0;
        const double arrive_sec =
            tdsp->debugPathArrivalTimeSec(path, depart_sec, wait_sec, travel_sec, static_sec);
        const double eta_sec = std::isfinite(arrive_sec) ? std::max(0.0, arrive_sec - depart_sec)
                                                         : std::numeric_limits<double>::infinity();
      CLOG(INFO, "mission.state_machine") 
            << "HSHMAT: Planned path time-to-goal (TDSP/FIFO): eta=" << eta_sec
            << "s (wait=" << wait_sec << "s, travel=" << travel_sec
            << "s, static_delay=" << static_sec << "s, nominal_speed="
            << tdsp->nominalSpeed() << " m/s)";
      }
    }

    CLOG(INFO, "mission.state_machine")
        << "Plan computed with " << path.size() << " vertices. From "
        << persistent_loc.v << " to " << waypoints_.front();
    tactic->setPath(path, 0, persistent_loc.T, true);
    
    // Hshmat: Update waypoints to match the new computed path for consistent following
    if (!path.empty()) {
      // Extract waypoints from the computed path (excluding start vertex)
      std::list<VertexId> new_waypoints;
      std::list<uint64_t> new_waypoint_seq;
      
      // Add intermediate vertices as waypoints (excluding start and end)
      for (size_t i = 1; i < path.size() - 1; ++i) {
        new_waypoints.push_back(path[i]);
        new_waypoint_seq.push_back(i); // Use path index as sequence
      }
      // Add the final goal
      if (!path.empty()) {
        new_waypoints.push_back(path.back());
        new_waypoint_seq.push_back(path.size() - 1);
      }
      
      // Update waypoints for the current state and parent states
      waypoints_ = new_waypoints;
      waypoint_seq_ = new_waypoint_seq;
      
      CLOG(INFO, "mission.state_machine") 
          << "HSHMAT: Updated waypoints to match new route: " << new_waypoints;
    }
  }

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onExit(state_machine, new_state);
}

void Plan::onEntry(StateMachine &state_machine, StateInterface &old_state) {
  // If the previous state was a derived class, we did not leave
  if (InChain(old_state) && !IsType(old_state)) return;

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onEntry(state_machine, old_state);

  // Note: This is called after we call up the tree, as we construct from root
  // to leaves
}

}  // namespace repeat
}  // namespace mission_planning
}  // namespace vtr