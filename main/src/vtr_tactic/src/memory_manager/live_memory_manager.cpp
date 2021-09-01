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
 * \file live_memory_manager.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_tactic/memory_manager/live_memory_manager.hpp>

namespace vtr {
namespace tactic {

bool LiveMemoryManager::checkUpdate() {
  if (tactic_->currentVertexID().isValid() &&
      tactic_->currentVertexID() != live_vertex_) {
    live_vertex_ = tactic_->currentVertexID();
    return true;
  }
  return false;
}

void LiveMemoryManager::manageMemory() {
  // if we have less than the desired size in memory then return.
  if (live_vertex_.minorId() < config_.window_size) return;

  // make sure the vertex to unload is on the same run as the live vertex.
  if (live_vertex_.majorId() != vertex_to_unload_.majorId())
    vertex_to_unload_ = VertexId(live_vertex_.majorId(), 0);

  // write and unload all vertices outside of the window
  while (vertex_to_unload_.minorId() + config_.window_size <
         live_vertex_.minorId()) {
    auto vertex = graph_->at(vertex_to_unload_);

    // write out the vertex data
    vertex->write();

    // are we currently localising on the same run?
    auto trunk_id = tactic_->closestVertexID();
    LOG(DEBUG) << "[Live Memory Manager] Closest vertex is : " << trunk_id;
    if (trunk_id != VertexId::Invalid() &&
        trunk_id.majorId() == vertex_to_unload_.majorId()) {
#if false
      // are we trying to unload a vertex close to the current trunk ID?
      unsigned diff =
          std::abs((int)vertex_to_unload_.minorId() - (int)trunk_id.minorId());
      // is the vertex in the localisation window?
      if (diff <= config_.lookahead_distance) {
        // don't unload it. Let the map memory manager clean it up
        vertex_to_unload_++;
        continue;
      }
#endif
      continue;  /// \todo we should be in merge mode at this moment?
    }

    // actually unload
    LOG(DEBUG)
        << "[Live Memory Manager] Unloading data associated with vertex: "
        << vertex;
    vertex->unload();
    vertex_to_unload_++;
  }
}

}  // namespace tactic
}  // namespace vtr
