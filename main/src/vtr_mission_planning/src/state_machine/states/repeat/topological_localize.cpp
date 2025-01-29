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
 * \file topological_localize.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_mission_planning/state_machine/states/repeat/topological_localize.hpp"
#include <fstream>
#include <sensor_msgs/msg/point_cloud2.hpp>


namespace fs = std::filesystem;
namespace vtr {
namespace mission_planning {
namespace repeat {


StateInterface::Ptr TopologicalLocalize::entryState() const { return nullptr; }

StateInterface::Ptr TopologicalLocalize::nextStep(
    const StateInterface &new_state) const {
  // If where we are going is not a child, delegate to the parent
  if (!InChain(new_state)) return Parent::nextStep(new_state);

  // If we aren't changing to a different chain, there is no intermediate step
  return nullptr;
}

// void TopologicalLocalize::processGoals(StateMachine &state_machine,
//                                        const Event &event) {
    
//     // int closest_vertex = getTactic(state_machine)->scan_matching();
//     // std::cerr << "closest_vertex NEW2: " << closest_vertex << std::endl;
//     // VertexId v_scan_match(0, closest_vertex); 
//     // getTactic(state_machine)->setTrunk(v_scan_match);



//   switch (event.signal) {
//     case Signal::Continue:
//       break;
//     default:
//       return Parent::processGoals(state_machine, event);
//   }

//     switch (event.action) {
    
//     case Action::Continue:
//         if (!getTactic(state_machine)->getPersistentLoc().v.isValid()) {
//         std::string err{"Attempted to repeat without a persistent loc set!"};
//         CLOG(WARNING, "mission.state_machine") << err;
//         throw std::runtime_error(err);
//         }
//           getTactic(state_machine)->registerScanMatch();

//         std::cerr <<"Vertex value in toploc: " << getTactic(state_machine)->getPersistentLoc().v << std::endl;

//         if (true) { //isLocalized()(getTactic(state_machine)->isLocalized() == true)

//             return Parent::processGoals(state_machine, Event(Action::EndGoal));
        
//         } else {
//         std::cerr << "INVALID VERTEX" << std::endl;
//         }
//         break;  

//     default:
//         return Parent::processGoals(state_machine, event);
//     }
// }

void TopologicalLocalize::processGoals(StateMachine &state_machine,
                                       const Event &event) {
  switch (event.signal) {
    case Signal::Continue:
      break;
    default:
      return Parent::processGoals(state_machine, event);
  }

  switch (event.action) {
    case Action::Continue:
      if (!getTactic(state_machine)->getPersistentLoc().v.isValid()) {
        std::string err{"Attempted to repeat without a persistent loc set!"};
        CLOG(WARNING, "mission.state_machine") << err;
        throw std::runtime_error(err);
      }
      /// \todo currently no topological localization, just go to Plan
      // If the closest vertex in the teach to the robot should be found automatically. Otherwise comment out and use the "Move Robot" tool in the GUI. 
      // getTactic(state_machine)->registerScanMatch();
      return Parent::processGoals(state_machine, Event(Action::EndGoal));
    default:
      return Parent::processGoals(state_machine, event);
  }
}

//Set tactic trunk to vertex id which we closest to. Check that trunk is valid
//Persistance loc id to invalid until a valid one is found (getPersistentLoc().v.isValid())
//Current pointcloud: On regular lidra module access through qdata (seperate thread), in qdata it says what the state of SM is. 
//Embeddings from teach are in folder 
//Calcuate distance
//Stay until valid vertex is found
//Once vertex found, update tactic (qdata.path/chain.settrunk and put vertex id there)
// ICP and stuff happens automatically 
// Set the local vertex, triggers SM to move on to metric localisation, then ICP starts running. 
// To get the tactic and set the vertex (maybe): getTactic(state_machine)->setTrunk(vertexid)



void TopologicalLocalize::onExit(StateMachine &state_machine,
                                 StateInterface &new_state) {
  // If the new target is a derived class, we are not exiting
  if (InChain(new_state) && !IsType(new_state)) return;

  // Note: This is called *before* we call up the tree, as we destruct from
  // leaves to root

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onExit(state_machine, new_state);
}

void TopologicalLocalize::onEntry(StateMachine &state_machine,
                                  StateInterface &old_state) {
  // If the previous state was a derived class, we did not leave
  if (InChain(old_state) && !IsType(old_state)) return;

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onEntry(state_machine, old_state);

  // Note: This is called after we call up the tree, as we construct from root
  // to leaves

  // Invalidate the getTactic(state_machine)->getPersistentLoc().v, otherway to invalidate:   tactic->setPath(match_window_, trunk_sid, tactic::EdgeTransform(true), true);

}

}  // namespace repeat
}  // namespace mission_planning
}  // namespace vtr
