#include <chrono>

#include <vtr/navigation/memory/live_memory_manager.h>

namespace vtr {
namespace navigation {

LiveMemoryManager::LiveMemoryManager(const std::shared_ptr<Graph> graph,
                                     BasicTactic *tactic,
                                     LiveMemoryManagerConfig &config)
    : graph_(graph),
      tactic_(tactic),
      config_(config),
      live_vertex_(0, 0),
      vertex_to_unload(0, 0),
      MemoryManager() {}

bool LiveMemoryManager::checkUpdate() {
  return tactic_->currentVertexID() != live_vertex_;
}

void LiveMemoryManager::manageMemory() {
  // update the live_id_
  live_vertex_ = tactic_->currentVertexID();

  // if we have less than the desired size in memory then return.
  if (live_vertex_.minorId() < (unsigned)config_.window_size) {
    return;
  }

  // make sure the vertex to unload is on the same run as the live vertex.
  if (live_vertex_.majorId() != vertex_to_unload.majorId()) {
    vertex_to_unload = VertexId(live_vertex_.majorId(), 0);
  }

  // write and unload all vertices outside of the window
  while (vertex_to_unload.minorId() + (unsigned)config_.window_size <
         live_vertex_.minorId()) {
    auto vertex = graph_->at(vertex_to_unload);

    // write out the vertex data
    vertex->write();

    // are we currently localising on the same run?
    auto trunk_id = tactic_->closestVertexID();
    if (trunk_id != VertexId::Invalid() &&
        trunk_id.majorId() == vertex_to_unload.majorId()) {
      // are we trying to unload a vertex close to the current trunk ID?
      unsigned diff =
          std::abs((int)vertex_to_unload.minorId() - (int)trunk_id.minorId());
      // is the vertex in the localisation window?
      if (diff <= (unsigned)config_.lookahead_distance) {
        // don't unload it. Let the map memory manager clean it up
        vertex_to_unload++;
        continue;
      }
    }

    // actually unload
    vertex->unload();
    vertex_to_unload++;
  }
}

}  // namespace navigation
}  // namespace vtr
