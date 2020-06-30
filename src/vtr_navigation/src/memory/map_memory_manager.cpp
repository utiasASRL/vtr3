#include <chrono>

#include <vtr/navigation/memory/map_memory_manager.h>

namespace vtr {
namespace navigation {

MapMemoryManager::MapMemoryManager(const std::shared_ptr<Graph> graph,
                                   const asrl::pose_graph::LocalizationChain &chain,
                                   MapMemoryManagerConfig &config)
    : graph_(graph), chain_(chain), config_(config), MemoryManager() {
  loaded_trunk_id_ = VertexId::Invalid();
}

bool MapMemoryManager::checkUpdate() {
  return chain_.sequence().size() > 0 &&
         chain_.trunkVertexId() != loaded_trunk_id_;
}

void MapMemoryManager::manageMemory() {
  loadVertices();
  unloadDeadVertices();
}

void MapMemoryManager::loadVertices() {
  // do a search out on the chain, up to the lookahead distance.
  PrivilegedEvaluatorPtr evaluator(new PrivilegedEvaluator());
  evaluator->setGraph(graph_.get());
  auto itr = graph_->beginDfs(chain_.trunkVertexId(),
                              config_.lookahead_distance, evaluator);
  try {
    for (; itr != graph_->end(); ++itr) {
      // load up the vertex and its spatial neighbors.
      auto vertex = itr->v();
      load(vertex, config_.priv_streams_to_load);
      load(vertex, config_.streams_to_load);
      for (auto &spatial_neighbor_id : vertex->spatialNeighbours()) {
        if (spatial_neighbor_id.majorId() != chain_.twigVertexId().majorId()) {
          auto neighbor = graph_->at(spatial_neighbor_id);
          load(neighbor, config_.streams_to_load);
        }
      }
    }
  } catch (std::range_error &e) {
    std::stringstream ss(e.what());
    std::string to;
    std::getline(ss, to, '\n');
    LOG(WARNING)
        << to << std::endl
        << "It's likely the vertex or edge was not yet inserted in the graph."
        << std::endl
        << "This should be OK if it doesn't happen repeatedly. Enable DEBUG to "
           "see details.";
    LOG(DEBUG) << e.what();
  }
  // set the loaded trunk id
  loaded_trunk_id_ = chain_.trunkVertexId();
}

void MapMemoryManager::load(std::shared_ptr<asrl::pose_graph::RCVertex> &vertex,
                            const std::vector<std::string> &streams_to_load) {
  // load up all of the requested streams.
  for (const auto &stream : streams_to_load) {
    vertex->load(stream);
  }
  // give the vertex a life span.
  life_map_[vertex->id()] = config_.vertex_life_span;
}

void MapMemoryManager::unloadDeadVertices() {
  // take life from all vertices.
  auto life = life_map_.begin();
  std::vector<decltype(life)> to_die;
  for (; life != life_map_.end(); ++life) {
    life->second--;

    // if the vertices life is zero, then sentence it to die.
    if (life->second <= 0) {
      auto vertex = graph_->at(life->first);
      vertex->unload();
      to_die.push_back(life);
    }
  }

  // kill vertices that are scheduled to die.
  for (auto &death : to_die) {
    life_map_.erase(death);
  }
}

}  // namespace navigation
}  // namespace vtr
