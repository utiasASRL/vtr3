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
 * \file real_world_logger.hpp
 * \brief Real-world episode and encounter logging for VTR3.
 * 
 * HSHMAT: Logs episode statistics and encounter details during real-world runs,
 * mirroring the simulation logging format for consistent analysis.
 */
#pragma once

#include <string>
#include <map>
#include <vector>
#include <fstream>
#include <mutex>

namespace vtr {
namespace navigation {

/**
 * \brief Logs real-world episode statistics and encounter records to CSV files.
 * 
 * Creates:
 * - episode_stats.csv: Per-episode summary (time_to_goal, success, reroutes, etc.)
 * - encounters_{STRATEGY}.csv: Per-encounter details (blocked_edge, W_star, decision, etc.)
 */
class RealWorldLogger {
 public:
  /**
   * \brief Record of a single obstacle encounter during an episode.
   */
  struct EncounterRecord {
    int episode = 0;
    std::string blocked_edge;   // "(v1,v2);(v3,v4)" format for multiple edges
    std::string obs_type;       // person, chair, sonotube, bin, unknown
    double W_star = 0.0;        // Wait time from policy
    std::string decision;       // "wait" or "reroute"
    double t_see = 0.0;         // Time obstacle was detected (seconds since mission start)
    double duration = 0.0;      // How long the encounter lasted
  };

  /**
   * \brief Statistics for a completed episode.
   */
  struct EpisodeStats {
    int episode = 0;
    std::string strategy;       // LEARNED, ORACLE, ALWAYS_WAIT, etc.
    double time_to_goal = 0.0;  // Total time to reach goal
    bool success = false;       // True if reached goal within time limit
    int reroutes = 0;           // Number of reroutes during episode
    double waiting_time = 0.0;  // Total time spent waiting
    std::map<std::string, int> encounters;  // Per-type encounter counts
  };

  RealWorldLogger() = default;
  ~RealWorldLogger();

  /**
   * \brief Initialize the logger with output directory and strategy name.
   * \param output_dir Directory where CSV files will be written
   * \param strategy Strategy name (e.g., "LEARNED") for encounters file naming
   * \return True if initialization successful
   */
  bool init(const std::string& output_dir, const std::string& strategy);

  /**
   * \brief Record an obstacle encounter.
   * \param rec Encounter record to log
   */
  void recordEncounter(const EncounterRecord& rec);

  /**
   * \brief Finalize an episode and write its statistics.
   * \param stats Episode statistics to log
   */
  void finalizeEpisode(const EpisodeStats& stats);

  /**
   * \brief Flush all output files to disk.
   */
  void flush();

  /**
   * \brief Check if logger is initialized.
   */
  bool isInitialized() const { return initialized_; }

  /**
   * \brief Get the output directory path.
   */
  const std::string& outputDir() const { return output_dir_; }

  /**
   * \brief Get the starting episode number (1 if new, or last_episode+1 if resuming).
   * Call after init() to get the correct starting episode.
   */
  int getStartingEpisode() const { return starting_episode_; }

 private:
  void writeEpisodeHeader();
  void writeEncounterHeader();
  int scanLastEpisode(const std::string& filepath);

  bool initialized_ = false;
  std::string output_dir_;
  std::string strategy_;
  int starting_episode_ = 1;  // Episode number to start from (handles resume)
  
  std::ofstream episode_file_;
  std::ofstream encounter_file_;
  
  mutable std::mutex mutex_;
  
  // Obstacle types for consistent column ordering
  static const std::vector<std::string> kObstacleTypes;
};

}  // namespace navigation
}  // namespace vtr
