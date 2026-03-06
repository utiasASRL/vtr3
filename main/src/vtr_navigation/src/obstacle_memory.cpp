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
 * \file obstacle_memory.cpp
 * \brief Implementation of ObstacleMemoryManager.
 * 
 * HSHMAT: Manages memory of obstacle episodes for learned strategy.
 */

#include "vtr_navigation/obstacle_memory.hpp"

namespace vtr {
namespace navigation {

void ObstacleMemoryManager::recordObstacle(
    const EdgeIdSet& blocked_edges,
    const std::string& obs_type,
    double t_now) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  EdgeObstacleMemory memory;
  memory.blocked_edges = blocked_edges;
  memory.obs_type = obs_type;
  memory.t_first = t_now;
  memory.t_last_confirmed = t_now;
  
  // Store same memory for all blocked edges
  for (const auto& edge : blocked_edges) {
    edge_memories_[edge] = memory;
  }
}

void ObstacleMemoryManager::updateAfterCensoredWait(const EdgeId& edge, 
                                                     double t_after_wait) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  auto it = edge_memories_.find(edge);
  if (it == edge_memories_.end()) return;
  
  // Update t_last_confirmed for all edges in the group
  const auto& blocked_edges = it->second.blocked_edges;
  for (const auto& e : blocked_edges) {
    auto edge_it = edge_memories_.find(e);
    if (edge_it != edge_memories_.end()) {
      edge_it->second.t_last_confirmed = t_after_wait;
    }
  }
}

void ObstacleMemoryManager::clearEdge(const EdgeId& edge) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  auto it = edge_memories_.find(edge);
  if (it == edge_memories_.end()) return;
  
  // Clear all edges in the same group
  auto blocked_edges = it->second.blocked_edges;  // Copy before erasing
  for (const auto& e : blocked_edges) {
    edge_memories_.erase(e);
  }
}

std::optional<EdgeObstacleMemory> ObstacleMemoryManager::getMemory(
    const EdgeId& edge) const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  auto it = edge_memories_.find(edge);
  if (it != edge_memories_.end()) {
    return it->second;
  }
  return std::nullopt;
}

bool ObstacleMemoryManager::hasMemory(const EdgeId& edge) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return edge_memories_.count(edge) > 0;
}

std::unordered_map<EdgeId, EdgeObstacleMemory, EdgeIdHash> 
ObstacleMemoryManager::getAllMemories() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return edge_memories_;
}

void ObstacleMemoryManager::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  edge_memories_.clear();
}

EdgeId ObstacleMemoryManager::findPrimaryEdge(const EdgeId& edge) const {
  // The "primary" edge is just the edge itself - used for consistent updates
  return edge;
}

}  // namespace navigation
}  // namespace vtr
