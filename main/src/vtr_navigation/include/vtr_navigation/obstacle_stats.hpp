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
 * \file obstacle_stats.hpp
 * \brief Global obstacle statistics for learned wait policy.
 * 
 * HSHMAT: Tracks statistics that improve with data collection:
 * - p_block: probability of obstacle on any edge
 * - p_obs_type: distribution over obstacle types
 * 
 * Data persists across runs in a location-specific directory.
 */
#pragma once

#include <string>
#include <map>
#include <mutex>

namespace vtr {
namespace navigation {

/**
 * \brief Global obstacle statistics for the learned policy.
 * 
 * Tracks:
 * - Total edges traversed (for p_block denominator)
 * - Total obstacle episodes (for p_block numerator)
 * - Count per obstacle type (for p_obs_type)
 * 
 * Persists to YAML file so statistics improve across runs.
 */
class GlobalObstacleStats {
 public:
  GlobalObstacleStats() = default;
  
  /**
   * \brief Load statistics from a YAML file.
   * \param path Path to the stats file
   * \return true if loaded successfully, false otherwise
   */
  bool loadFromFile(const std::string& path);
  
  /**
   * \brief Save statistics to a YAML file.
   * \param path Path to the stats file
   * \return true if saved successfully, false otherwise
   */
  bool saveToFile(const std::string& path) const;
  
  /**
   * \brief Record that an edge was traversed.
   * Called for each edge the robot drives over.
   */
  void recordEdgeTraversal();
  
  /**
   * \brief Record that an edge was traversed (batch).
   * \param count Number of edges traversed
   */
  void recordEdgeTraversals(int count);
  
  /**
   * \brief Record an obstacle episode.
   * \param obs_type Obstacle type (e.g., "person", "chair")
   */
  void recordObstacleEpisode(const std::string& obs_type);
  
  /**
   * \brief Get probability of obstacle on any edge.
   * \return p_block = total_obstacle_episodes / total_edges_traversed
   */
  double p_block() const;
  
  /**
   * \brief Get probability of a specific obstacle type.
   * \param obs_type Obstacle type
   * \return p(obs_type) = count[obs_type] / total_obstacle_episodes
   */
  double p_obs_type(const std::string& obs_type) const;
  
  /**
   * \brief Get all obstacle types and their probabilities.
   * \return Map of type -> probability
   */
  std::map<std::string, double> getTypeDistribution() const;
  
  /**
   * \brief Get total edges traversed.
   */
  int totalEdgesTraversed() const { return total_edges_traversed_; }
  
  /**
   * \brief Get total obstacle episodes.
   */
  int totalObstacleEpisodes() const { return total_obstacle_episodes_; }
  
  /**
   * \brief Get count for a specific obstacle type.
   */
  int typeCount(const std::string& obs_type) const;
  
  /**
   * \brief Check if we have enough data for reliable estimates.
   * \param min_edges Minimum edges traversed
   * \param min_episodes Minimum obstacle episodes
   * \return true if we have enough data
   */
  bool hasEnoughData(int min_edges = 100, int min_episodes = 5) const;
  
  /**
   * \brief Clear all statistics (for testing).
   */
  void clear();
  
  /**
   * \brief Set default p_block to use when we don't have enough data.
   */
  void setDefaultPBlock(double p) { default_p_block_ = p; }
  
  /**
   * \brief Set default type weights to use when we don't have enough data.
   */
  void setDefaultTypeWeights(const std::map<std::string, double>& weights) {
    default_type_weights_ = weights;
  }

 private:
  mutable std::mutex mutex_;
  
  int total_edges_traversed_ = 0;
  int total_obstacle_episodes_ = 0;
  std::map<std::string, int> type_counts_;
  
  // Defaults to use when we don't have enough data
  double default_p_block_ = -1.0;  // Must be set via setDefaultPBlock() from YAML config
  std::map<std::string, double> default_type_weights_;
};

}  // namespace navigation
}  // namespace vtr
