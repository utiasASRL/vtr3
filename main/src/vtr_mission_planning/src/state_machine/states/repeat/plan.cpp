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

    auto bfs = std::dynamic_pointer_cast<vtr::route_planning::BFSPlanner>(route_planner);
    PathType path;
    const bool using_masked = bfs && bfs->useMasked();
    
    CLOG(INFO, "mission.state_machine") << "HSHMAT: Plan::onExit - Starting route computation";
    CLOG(INFO, "mission.state_machine") << "HSHMAT: Using masked planning: " << (using_masked ? "true" : "false");
    
    if (using_masked) {
      // Use masked Dijkstra honoring banned edges; if it fails (e.g., no route
      // exists under current mask), log and fall back to unmasked planning.
      CLOG(INFO, "mission.state_machine")
          << "HSHMAT: Attempting masked plan from " << persistent_loc.v << " to "
          << waypoints_.front() << ", permanentBan="
          << (bfs->permanentBan() ? "true" : "false");
      try {
        path = bfs->hshmat_plan(persistent_loc.v, waypoints_.front());
        CLOG(INFO, "mission.state_machine")
            << "HSHMAT: Masked plan succeeded with " << path.size() << " vertices";

        // Debug: report total time cost (travel time + extra delay) of the
        // masked path. This uses the same cost model as deterministic_timecost:
        // distance/nominal_speed + per-edge extra delays.
        CLOG(INFO, "mission.state_machine")
            << "HSHMAT: DEBUG - Checking path cost (bfs=" << (bfs ? "valid" : "null") 
            << ", path.size()=" << path.size() << ")";
        if (bfs && !path.empty()) {
          double delay_sec = 0.0;
          const double total_cost = bfs->debugPathTotalCost(path, delay_sec);
          CLOG(INFO, "mission.state_machine")
              << "HSHMAT: ========== PATH COST ANALYSIS ==========";
          CLOG(INFO, "mission.state_machine")
              << "HSHMAT: Masked path total cost=" << total_cost
              << " s (base travel time + extra delay=" << delay_sec
              << " s, nominal_speed=" << bfs->nominalSpeed()
              << " m/s, use_time_cost=" << (bfs->useTimeCost() ? "true" : "false")
              << ")";
          CLOG(INFO, "mission.state_machine")
              << "HSHMAT: Path has " << path.size() << " vertices from "
              << persistent_loc.v << " to " << waypoints_.front();
          if (delay_sec > 0.0) {
            CLOG(WARNING, "mission.state_machine")
                << "HSHMAT: ⚠️  Path includes " << delay_sec 
                << "s of extra delay penalties!";
          } else {
            CLOG(INFO, "mission.state_machine")
                << "HSHMAT: ✓ Path has no extra delays (all edges clean)";
          }
          CLOG(INFO, "mission.state_machine")
              << "HSHMAT: =========================================";
        }

        if (auto cb = getCallback(state_machine))
          cb->notifyRerouteStatus("reroute_success");
      } catch (const std::exception &e) {
        CLOG(WARNING, "mission.state_machine")
            << "HSHMAT: Masked plan failed: '" << e.what()
            << "'. Falling back to unmasked route computation.";
        if (auto cb = getCallback(state_machine))
          cb->notifyRerouteStatus("reroute_mask_failed");
        try {
          path = route_planner->path(persistent_loc.v, waypoints_, waypoint_seq_);
          CLOG(INFO, "mission.state_machine") << "HSHMAT: Unmasked fallback succeeded with " << path.size() << " vertices";
          if (auto cb = getCallback(state_machine))
            cb->notifyRerouteStatus("reroute_fallback_used");
        } catch (const std::exception &e2) {
          CLOG(ERROR, "mission.state_machine")
              << "HSHMAT: Unmasked plan also failed: '" << e2.what()
              << "'. Keeping previous path; skipping setPath.";
          if (auto cb = getCallback(state_machine))
            cb->notifyRerouteStatus("reroute_failed");
          // Do not throw; leave without updating path to avoid crashing
          if (bfs && !bfs->permanentBan()) bfs->clearBannedEdges();
          Parent::onExit(state_machine, new_state);
          return;
        }
      }
    } else {
      CLOG(INFO, "mission.state_machine") << "HSHMAT: Using unmasked planning";
      path = route_planner->path(persistent_loc.v, waypoints_, waypoint_seq_);
      CLOG(INFO, "mission.state_machine") << "HSHMAT: Unmasked plan completed with " << path.size() << " vertices";
    }
    // Clear bans only if not permanent (reroute disabled)
    // Note: For rerouting, we keep permanent bans but clear them after successful planning
    // to allow future rerouting to work properly
    if (bfs && !bfs->permanentBan()) {
      CLOG(INFO, "mission.state_machine") 
          << "HSHMAT-DEBUG: Clearing banned edges (permanentBan=false). "
          << "Extra edge costs BEFORE clear: " << (bfs ? "checking..." : "bfs is null");
      bfs->clearBannedEdges();
      // NOTE: We do NOT clear extra_edge_costs here - they should persist for deterministic_timecost
      CLOG(INFO, "mission.state_machine") 
          << "HSHMAT-DEBUG: Banned edges cleared. Extra edge costs should still be set.";
    } else if (bfs && bfs->permanentBan()) {
      // If permanentBan() is true, we keep banned edges across replans.
      CLOG(INFO, "mission.state_machine") << "HSHMAT: Permanent ban is enabled, not clearing banned edges";
      // bfs->clearBannedEdges();
    }

    CLOG(INFO, "mission.state_machine")
        << "Plan computed (masked=" << (using_masked ? "true" : "false")
        << ") with " << path.size() << " vertices. From " << persistent_loc.v
        << " to " << waypoints_.front();
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