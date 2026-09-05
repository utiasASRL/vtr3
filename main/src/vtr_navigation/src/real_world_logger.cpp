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
 * \file real_world_logger.cpp
 * \brief Implementation of real-world episode and encounter logging.
 */

#include "vtr_navigation/real_world_logger.hpp"

#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>

namespace vtr {
namespace navigation {

// Canonical obstacle types for CSV columns
const std::vector<std::string> RealWorldLogger::kObstacleTypes = {
    "person", "chair", "sonotube", "bin"
};

RealWorldLogger::~RealWorldLogger() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (episode_file_.is_open()) {
    episode_file_.flush();
    episode_file_.close();
  }
  if (encounter_file_.is_open()) {
    encounter_file_.flush();
    encounter_file_.close();
  }
}

int RealWorldLogger::scanLastEpisode(const std::string& filepath) {
  // Scan a CSV file to find the highest episode number
  std::ifstream file(filepath);
  if (!file.is_open()) {
    return 0;  // File doesn't exist
  }
  
  int max_episode = 0;
  std::string line;
  bool first_line = true;
  
  while (std::getline(file, line)) {
    if (first_line) {
      first_line = false;  // Skip header
      continue;
    }
    if (line.empty()) continue;
    
    // Extract episode number (first column)
    size_t comma_pos = line.find(',');
    if (comma_pos != std::string::npos) {
      try {
        int episode = std::stoi(line.substr(0, comma_pos));
        max_episode = std::max(max_episode, episode);
      } catch (...) {
        // Ignore parsing errors
      }
    }
  }
  
  file.close();
  return max_episode;
}

bool RealWorldLogger::init(const std::string& output_dir, const std::string& strategy) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (initialized_) {
    std::cerr << "RealWorldLogger: Already initialized" << std::endl;
    return false;
  }
  
  output_dir_ = output_dir;
  strategy_ = strategy;
  
  // Create output directory if it doesn't exist
  struct stat st;
  if (stat(output_dir_.c_str(), &st) != 0) {
    // Directory doesn't exist, create it recursively
    std::string cmd = "mkdir -p " + output_dir_;
    int ret = system(cmd.c_str());
    if (ret != 0) {
      std::cerr << "RealWorldLogger: Failed to create directory: " << output_dir_ << std::endl;
      return false;
    }
  }
  
  std::string episode_path = output_dir_ + "/episode_stats.csv";
  std::string encounter_path = output_dir_ + "/encounters_" + strategy_ + ".csv";
  
  // Check if files exist and scan for last episode number
  bool episode_file_exists = (stat(episode_path.c_str(), &st) == 0);
  bool encounter_file_exists = (stat(encounter_path.c_str(), &st) == 0);
  
  int last_episode_from_stats = 0;
  int last_episode_from_encounters = 0;
  
  if (episode_file_exists) {
    last_episode_from_stats = scanLastEpisode(episode_path);
  }
  if (encounter_file_exists) {
    last_episode_from_encounters = scanLastEpisode(encounter_path);
  }
  
  // Use the maximum episode found from either file
  int last_episode = std::max(last_episode_from_stats, last_episode_from_encounters);
  starting_episode_ = last_episode + 1;
  
  // Open files - append if they exist, create with header if new
  if (episode_file_exists && last_episode > 0) {
    // Append mode - file has data
    episode_file_.open(episode_path, std::ios::out | std::ios::app);
    if (!episode_file_.is_open()) {
      std::cerr << "RealWorldLogger: Failed to open " << episode_path << std::endl;
      return false;
    }
    std::cout << "RealWorldLogger: Resuming episode_stats.csv (last episode: " 
              << last_episode_from_stats << ")" << std::endl;
  } else {
    // Truncate mode - new file
    episode_file_.open(episode_path, std::ios::out | std::ios::trunc);
    if (!episode_file_.is_open()) {
      std::cerr << "RealWorldLogger: Failed to open " << episode_path << std::endl;
      return false;
    }
    writeEpisodeHeader();
  }
  
  if (encounter_file_exists && last_episode > 0) {
    // Append mode - file has data
    encounter_file_.open(encounter_path, std::ios::out | std::ios::app);
    if (!encounter_file_.is_open()) {
      std::cerr << "RealWorldLogger: Failed to open " << encounter_path << std::endl;
      episode_file_.close();
      return false;
    }
    std::cout << "RealWorldLogger: Resuming encounters CSV (last episode: " 
              << last_episode_from_encounters << ")" << std::endl;
  } else {
    // Truncate mode - new file
    encounter_file_.open(encounter_path, std::ios::out | std::ios::trunc);
    if (!encounter_file_.is_open()) {
      std::cerr << "RealWorldLogger: Failed to open " << encounter_path << std::endl;
      episode_file_.close();
      return false;
    }
    writeEncounterHeader();
  }
  
  initialized_ = true;
  std::cout << "RealWorldLogger: Initialized at " << output_dir_ 
            << " (strategy=" << strategy_ << ", starting_episode=" << starting_episode_ << ")" << std::endl;
  return true;
}

void RealWorldLogger::writeEpisodeHeader() {
  // Write CSV header for episode_stats.csv
  episode_file_ << "episode,strategy,time_to_goal,success,reroutes,waiting_time";
  for (const auto& obs_type : kObstacleTypes) {
    episode_file_ << ",encounters_" << obs_type;
  }
  episode_file_ << "\n";
  episode_file_.flush();
}

void RealWorldLogger::writeEncounterHeader() {
  // Write CSV header for encounters_{strategy}.csv
  encounter_file_ << "episode,blocked_edge,obs_type,W_star,decision,t_see,duration\n";
  encounter_file_.flush();
}

void RealWorldLogger::recordEncounter(const EncounterRecord& rec) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!initialized_ || !encounter_file_.is_open()) {
    std::cerr << "RealWorldLogger: Not initialized, cannot record encounter" << std::endl;
    return;
  }
  
  // Write encounter row
  encounter_file_ << rec.episode << ","
                  << "\"" << rec.blocked_edge << "\","
                  << rec.obs_type << ","
                  << std::fixed << std::setprecision(2) << rec.W_star << ","
                  << rec.decision << ","
                  << std::fixed << std::setprecision(3) << rec.t_see << ","
                  << std::fixed << std::setprecision(3) << rec.duration << "\n";
  encounter_file_.flush();
  
  std::cout << "RealWorldLogger: Recorded encounter - episode=" << rec.episode
            << ", type=" << rec.obs_type << ", decision=" << rec.decision
            << ", duration=" << rec.duration << "s" << std::endl;
}

void RealWorldLogger::finalizeEpisode(const EpisodeStats& stats) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!initialized_ || !episode_file_.is_open()) {
    std::cerr << "RealWorldLogger: Not initialized, cannot finalize episode" << std::endl;
    return;
  }
  
  // Write episode row
  episode_file_ << stats.episode << ","
                << stats.strategy << ","
                << std::fixed << std::setprecision(2) << stats.time_to_goal << ","
                << (stats.success ? 1 : 0) << ","
                << stats.reroutes << ","
                << std::fixed << std::setprecision(2) << stats.waiting_time;
  
  // Write encounter counts for each obstacle type
  for (const auto& obs_type : kObstacleTypes) {
    auto it = stats.encounters.find(obs_type);
    int count = (it != stats.encounters.end()) ? it->second : 0;
    episode_file_ << "," << count;
  }
  episode_file_ << "\n";
  episode_file_.flush();
  
  std::cout << "RealWorldLogger: Finalized episode " << stats.episode
            << " - time=" << stats.time_to_goal << "s, success=" << stats.success
            << ", reroutes=" << stats.reroutes << ", wait_time=" << stats.waiting_time << "s"
            << std::endl;
}

void RealWorldLogger::flush() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (episode_file_.is_open()) {
    episode_file_.flush();
  }
  if (encounter_file_.is_open()) {
    encounter_file_.flush();
  }
}

}  // namespace navigation
}  // namespace vtr
