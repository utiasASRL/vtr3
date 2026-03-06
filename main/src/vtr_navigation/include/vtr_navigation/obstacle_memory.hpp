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
 * \file obstacle_memory.hpp
 * \brief Memory management for obstacle episodes - tracks obstacles we've seen
 *        and given up on (censored waits).
 * 
 * HSHMAT: This implements the memory system from Python simulation.
 * When we encounter an obstacle and give up waiting (censored), we remember:
 * - Which edges are blocked
 * - What type of obstacle it was
 * - When we first saw it
 * - When we last confirmed it was still there
 * 
 * This memory is used to compute expected wait times for planning.
 */
#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <optional>

#include "vtr_tactic/types.hpp"

namespace vtr {
namespace navigation {

// Hash for unordered_set of EdgeId
struct EdgeIdHash {
  std::size_t operator()(const tactic::EdgeId& e) const {
    return e.hash();
  }
};

using EdgeIdSet = std::unordered_set<tactic::EdgeId, EdgeIdHash>;
using EdgeId = tactic::EdgeId;

/**
 * \brief Memory of a single obstacle episode on one or more edges.
 * 
 * When an obstacle blocks multiple edges (e.g., a person standing at an
 * intersection), we treat them as ONE obstacle - they all clear together.
 */
struct EdgeObstacleMemory {
  EdgeIdSet blocked_edges;   // All edges blocked by this obstacle
  std::string obs_type;      // "person", "chair", etc.
  double t_first;            // Global time when obstacle was first observed
  double t_last_confirmed;   // Global time of last confirmation (after censored wait)
  
  /**
   * \brief Confirmed duration the obstacle has persisted.
   * c = t_last_confirmed - t_first
   */
  double c() const { return t_last_confirmed - t_first; }
};

/**
 * \brief Manages memory of obstacle episodes for TDSP planning.
 * 
 * Provides:
 * - Recording of obstacle observations
 * - Updating after censored waits (we gave up, obstacle still there)
 * - Clearing after obstacle actually moves
 * - Lookup for expected wait computation
 */
class ObstacleMemoryManager {
 public:
  
  ObstacleMemoryManager() = default;
  
  /**
   * \brief Record a new obstacle encounter.
   * 
   * Called when robot first encounters a blocked edge. If obstacle blocks
   * multiple edges, call once with all edges.
   * 
   * \param blocked_edges Set of edges blocked by this obstacle
   * \param obs_type Obstacle type ("person", "chair", etc.)
   * \param t_now Current global time
   */
  void recordObstacle(const EdgeIdSet& blocked_edges,
                      const std::string& obs_type,
                      double t_now);
  
  /**
   * \brief Update memory after a censored wait (we gave up, obstacle still there).
   * 
   * Updates t_last_confirmed for all edges associated with this obstacle.
   * 
   * \param edge Any edge from the obstacle group
   * \param t_after_wait Global time after waiting (t0 + W)
   */
  void updateAfterCensoredWait(const EdgeId& edge, double t_after_wait);
  
  /**
   * \brief Clear memory for an edge (obstacle moved away).
   * 
   * Clears memory for ALL edges in the same obstacle group.
   * 
   * \param edge Any edge from the obstacle group
   */
  void clearEdge(const EdgeId& edge);
  
  /**
   * \brief Get memory for an edge if it exists.
   * \return Memory if edge has active memory, nullopt otherwise
   */
  std::optional<EdgeObstacleMemory> getMemory(const EdgeId& edge) const;
  
  /**
   * \brief Check if an edge has active memory.
   */
  bool hasMemory(const EdgeId& edge) const;
  
  /**
   * \brief Get all active memories.
   */
  std::unordered_map<EdgeId, EdgeObstacleMemory, EdgeIdHash> getAllMemories() const;
  
  /**
   * \brief Reset all memory (called at start of new Repeat).
   */
  void reset();

 private:
  mutable std::mutex mutex_;
  
  // Map from edge -> memory. All edges in the same obstacle group point to
  // the same memory object (via shared_ptr or copy).
  std::unordered_map<EdgeId, EdgeObstacleMemory, EdgeIdHash> edge_memories_;
  
  // Find the "primary" edge for an obstacle group (for consistent updates)
  EdgeId findPrimaryEdge(const EdgeId& edge) const;
};

}  // namespace navigation
}  // namespace vtr
