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
 * \file obstacle_stats.cpp
 * \brief Implementation of GlobalObstacleStats with YAML persistence.
 */
#include "vtr_navigation/obstacle_stats.hpp"

#include <fstream>
#include <filesystem>

#include "vtr_logging/logging.hpp"

// Simple YAML writing (avoid dependency on yaml-cpp for this simple structure)
namespace {

std::string escapeYamlString(const std::string& s) {
  // Simple escape for YAML strings
  if (s.find(':') != std::string::npos || s.find('#') != std::string::npos ||
      s.find('"') != std::string::npos || s.find('\'') != std::string::npos) {
    std::string escaped = "\"";
    for (char c : s) {
      if (c == '"') escaped += "\\\"";
      else if (c == '\\') escaped += "\\\\";
      else escaped += c;
    }
    escaped += "\"";
    return escaped;
  }
  return s;
}

}  // namespace

namespace vtr {
namespace navigation {

bool GlobalObstacleStats::loadFromFile(const std::string& path) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  std::ifstream file(path);
  if (!file.is_open()) {
    CLOG(INFO, "navigation") << "HSHMAT GlobalObstacleStats: No existing stats file at " << path;
    return false;
  }
  
  // Simple YAML parsing for our specific format
  std::string line;
  bool in_type_counts = false;
  
  while (std::getline(file, line)) {
    // Skip empty lines and comments
    if (line.empty() || line[0] == '#') continue;
    
    // Remove leading/trailing whitespace
    size_t start = line.find_first_not_of(" \t");
    size_t end = line.find_last_not_of(" \t");
    if (start == std::string::npos) continue;
    line = line.substr(start, end - start + 1);
    
    // Check for section marker
    if (line == "type_counts:") {
      in_type_counts = true;
      continue;
    }
    
    // Parse key: value
    size_t colon = line.find(':');
    if (colon == std::string::npos) continue;
    
    std::string key = line.substr(0, colon);
    std::string value = line.substr(colon + 1);
    
    // Trim whitespace from key and value
    key.erase(0, key.find_first_not_of(" \t"));
    key.erase(key.find_last_not_of(" \t") + 1);
    value.erase(0, value.find_first_not_of(" \t"));
    value.erase(value.find_last_not_of(" \t") + 1);
    
    if (in_type_counts) {
      // Type count entry (indented)
      if (key.empty() || value.empty()) continue;
      try {
        type_counts_[key] = std::stoi(value);
      } catch (...) {
        CLOG(WARNING, "navigation") << "HSHMAT GlobalObstacleStats: Failed to parse type count: " << line;
      }
    } else {
      // Top-level entry
      if (key == "total_edges_traversed") {
        try {
          total_edges_traversed_ = std::stoi(value);
        } catch (...) {
          CLOG(WARNING, "navigation") << "HSHMAT GlobalObstacleStats: Failed to parse total_edges_traversed";
        }
      } else if (key == "total_obstacle_episodes") {
        try {
          total_obstacle_episodes_ = std::stoi(value);
        } catch (...) {
          CLOG(WARNING, "navigation") << "HSHMAT GlobalObstacleStats: Failed to parse total_obstacle_episodes";
        }
      }
    }
  }
  
  CLOG(INFO, "navigation") << "HSHMAT GlobalObstacleStats: Loaded from " << path
                           << " - edges=" << total_edges_traversed_
                           << ", episodes=" << total_obstacle_episodes_
                           << ", types=" << type_counts_.size();
  return true;
}

bool GlobalObstacleStats::saveToFile(const std::string& path) const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  // Create directory if it doesn't exist
  std::filesystem::path filepath(path);
  std::filesystem::path dir = filepath.parent_path();
  if (!dir.empty() && !std::filesystem::exists(dir)) {
    try {
      std::filesystem::create_directories(dir);
      CLOG(INFO, "navigation") << "HSHMAT GlobalObstacleStats: Created directory " << dir;
    } catch (const std::exception& e) {
      CLOG(ERROR, "navigation") << "HSHMAT GlobalObstacleStats: Failed to create directory " << dir << ": " << e.what();
      return false;
    }
  }
  
  std::ofstream file(path);
  if (!file.is_open()) {
    CLOG(ERROR, "navigation") << "HSHMAT GlobalObstacleStats: Failed to open " << path << " for writing";
    return false;
  }
  
  file << "# Global obstacle statistics for learned wait policy\n";
  file << "# Auto-generated - do not edit manually\n\n";
  file << "total_edges_traversed: " << total_edges_traversed_ << "\n";
  file << "total_obstacle_episodes: " << total_obstacle_episodes_ << "\n";
  file << "\ntype_counts:\n";
  for (const auto& kv : type_counts_) {
    file << "  " << escapeYamlString(kv.first) << ": " << kv.second << "\n";
  }
  
  CLOG(DEBUG, "navigation") << "HSHMAT GlobalObstacleStats: Saved to " << path;
  return true;
}

void GlobalObstacleStats::recordEdgeTraversal() {
  std::lock_guard<std::mutex> lock(mutex_);
  ++total_edges_traversed_;
}

void GlobalObstacleStats::recordEdgeTraversals(int count) {
  std::lock_guard<std::mutex> lock(mutex_);
  total_edges_traversed_ += count;
}

void GlobalObstacleStats::recordObstacleEpisode(const std::string& obs_type) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++total_obstacle_episodes_;
  ++type_counts_[obs_type];
  
  CLOG(INFO, "navigation") << "HSHMAT GlobalObstacleStats: Recorded episode for '" << obs_type
                           << "' - total_episodes=" << total_obstacle_episodes_
                           << ", type_count=" << type_counts_[obs_type];
}

double GlobalObstacleStats::p_block() const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  // Use default if not enough data
  if (total_edges_traversed_ < 100 || total_obstacle_episodes_ < 5) {
    return default_p_block_;
  }
  
  return static_cast<double>(total_obstacle_episodes_) / total_edges_traversed_;
}

double GlobalObstacleStats::p_obs_type(const std::string& obs_type) const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  // Use default if not enough data
  if (total_obstacle_episodes_ < 5) {
    auto it = default_type_weights_.find(obs_type);
    if (it != default_type_weights_.end()) {
      return it->second;
    }
    // Uniform over known types
    if (!default_type_weights_.empty()) {
      return 1.0 / default_type_weights_.size();
    }
    return 0.2;  // Fallback
  }
  
  auto it = type_counts_.find(obs_type);
  if (it == type_counts_.end()) {
    return 0.0;
  }
  
  return static_cast<double>(it->second) / total_obstacle_episodes_;
}

std::map<std::string, double> GlobalObstacleStats::getTypeDistribution() const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  std::map<std::string, double> dist;
  
  // Use default if not enough data
  if (total_obstacle_episodes_ < 5) {
    return default_type_weights_;
  }
  
  for (const auto& kv : type_counts_) {
    dist[kv.first] = static_cast<double>(kv.second) / total_obstacle_episodes_;
  }
  
  return dist;
}

int GlobalObstacleStats::typeCount(const std::string& obs_type) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = type_counts_.find(obs_type);
  return (it != type_counts_.end()) ? it->second : 0;
}

bool GlobalObstacleStats::hasEnoughData(int min_edges, int min_episodes) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return total_edges_traversed_ >= min_edges && total_obstacle_episodes_ >= min_episodes;
}

void GlobalObstacleStats::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  total_edges_traversed_ = 0;
  total_obstacle_episodes_ = 0;
  type_counts_.clear();
}

}  // namespace navigation
}  // namespace vtr
