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
 * \file map_memory_manager.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_tactic/memory_manager/map_memory_manager.hpp>

namespace vtr {
namespace tactic {

bool MapMemoryManager::checkUpdate() {
  const auto lock = chain_->guard();
  if (chain_->sequence().size() > 0 && chain_->trunkVertexId().isValid() &&
      chain_->trunkVertexId() != trunk_id_) {
    trunk_id_ = chain_->trunkVertexId();
    twig_id_ = chain_->twigVertexId();
    return true;
  }
  return false;
}

void MapMemoryManager::manageMemory() {
  loadVertices();
  unloadDeadVertices();
}

void MapMemoryManager::loadVertices() {
  // do a search out on the chain, up to the lookahead distance.
  PrivilegedEvaluator::Ptr evaluator(new PrivilegedEvaluator());
  evaluator->setGraph(graph_.get());
  // search with the lock to get vertices
  std::vector<Graph::VertexPtr> vertices;
  graph_->lock();
  auto itr = graph_->beginDfs(trunk_id_, config_.lookahead_distance, evaluator);
  for (; itr != graph_->end(); ++itr) vertices.push_back(itr->v());
  graph_->unlock();
  try {
    /// \todo (yuchen) This part of code has not been tested!
    for (auto &vertex : vertices) {
      // load up the vertex and its spatial neighbors.
      load(vertex, config_.priv_streams_to_load);
      load(vertex, config_.streams_to_load);
      for (auto &spatial_neighbor_id : vertex->spatialNeighbours()) {
        if (spatial_neighbor_id.majorId() != twig_id_.majorId()) {
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
}

void MapMemoryManager::load(Vertex::Ptr &vertex,
                            const std::vector<std::string> &streams_to_load) {
  // load up all of the requested streams.
  for (const auto &stream : streams_to_load) vertex->load(stream);

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
      LOG(DEBUG)
          << "[Map Memory Manager] Unloading data associated with vertex: "
          << vertex->id();
      vertex->unload();
      to_die.push_back(life);
    }
  }

  // kill vertices that are scheduled to die.
  for (auto &death : to_die) life_map_.erase(death);
}

}  // namespace tactic
}  // namespace vtr
