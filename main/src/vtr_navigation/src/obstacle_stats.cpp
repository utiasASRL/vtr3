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
#include <algorithm>
#include "vtr_navigation/obstacle_stats.hpp"

#include <fstream>
#include <filesystem>
#include <set>

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

double GlobalObstacleStats::pBlockJeffreys() const {
  std::lock_guard<std::mutex> lock(mutex_);
  const double n = static_cast<double>(total_edges_traversed_);
  const double k = static_cast<double>(total_obstacle_episodes_);
  return std::min(1.0, std::max(0.0, (k + 0.5) / (n + 1.0)));
}

void GlobalObstacleStats::applyTeachPrior(int n) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (teach_prior_applied_ || n <= 0) return;
  teach_prior_applied_ = true;
  // Only seed a COLD start. total_edges_traversed_ is persisted to
  // obstacle_stats.yaml, so a resumed deployment has already banked the prior
  // in its saved counters - adding it again on every restart would inflate the
  // denominator without bound and drive the learned occupancy to zero. The
  // in-memory flag alone cannot catch that, because it is not persisted.
  if (total_edges_traversed_ > 0) return;
  total_edges_traversed_ += n;
}

void GlobalObstacleStats::recordObstacleEpisode(const std::string& obs_type) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++total_obstacle_episodes_;
  ++type_counts_[obs_type];
  
  CLOG(INFO, "navigation") << "HSHMAT GlobalObstacleStats: Recorded episode for '" << obs_type
                           << "' - total_episodes=" << total_obstacle_episodes_
                           << ", type_count=" << type_counts_[obs_type];
}

void GlobalObstacleStats::recordUnlabeledEpisode() {
  std::lock_guard<std::mutex> lock(mutex_);
  ++total_obstacle_episodes_;
  CLOG(INFO, "navigation")
      << "HSHMAT GlobalObstacleStats: Recorded UNLABELED episode - "
         "total_episodes="
      << total_obstacle_episodes_ << " (class mixture unchanged)";
}

double GlobalObstacleStats::p_block() const {
  std::lock_guard<std::mutex> lock(mutex_);
  // Always computed from data: episodes / edges, with 0/0 := 0.
  if (total_edges_traversed_ <= 0) return 0.0;
  return static_cast<double>(total_obstacle_episodes_) / total_edges_traversed_;
}

double GlobalObstacleStats::p_obs_type(const std::string& obs_type) const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  // Match simulation's Dirichlet prior logic exactly:
  // P(type) = (count + alpha) / (total + n_types * alpha)
  const double alpha = 1.0;
  
  // Get all known types
  std::set<std::string> all_types;
  for (const auto& kv : default_type_weights_) {
    all_types.insert(kv.first);
  }
  for (const auto& kv : type_counts_) {
    all_types.insert(kv.first);
  }
  
  if (all_types.empty()) {
    return 0.25;  // Fallback: assume 4 types
  }
  
  int n_types = static_cast<int>(all_types.size());
  double total_with_prior = static_cast<double>(total_obstacle_episodes_) + n_types * alpha;
  
  auto it = type_counts_.find(obs_type);
  double count = (it != type_counts_.end()) ? static_cast<double>(it->second) : 0.0;
  
  return (count + alpha) / total_with_prior;
}

std::map<std::string, double> GlobalObstacleStats::getTypeDistribution() const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  std::map<std::string, double> dist;
  
  // Dirichlet prior, but pointed at the CONFIGURED class mixture rather than
  // at uniform:
  //   P(type) = (count_k + alpha * n_types * w_k) / (total + n_types * alpha)
  // with w_k the normalised default_type_weights_ (uniform when none are set,
  // which reproduces the previous alpha=1 behaviour exactly).
  //
  // With no observations this returns w_k, which is what the deployment
  // configures it to be; with many observations the counts dominate and it
  // converges to the empirical mixture, as before. The old form returned a
  // flat 1/n at a cold start no matter what was configured, so a scenario
  // whose true mixture is 20/80 was planned as 50/50 -- the configured
  // weights only ever selected the class UNIVERSE and never its shape.
  const double alpha = 1.0;
  double weight_total = 0.0;
  for (const auto& kv : default_type_weights_)
    weight_total += std::max(0.0, kv.second);
  
  // Get all known types from default_type_weights_ (defines the type universe)
  std::set<std::string> all_types;
  for (const auto& kv : default_type_weights_) {
    all_types.insert(kv.first);
  }
  // Also include any types we've actually seen
  for (const auto& kv : type_counts_) {
    all_types.insert(kv.first);
  }
  
  if (all_types.empty()) {
    return dist;  // No types known
  }
  
  int n_types = static_cast<int>(all_types.size());
  double total_with_prior = static_cast<double>(total_obstacle_episodes_) + n_types * alpha;
  
  for (const auto& obs_type : all_types) {
    auto it = type_counts_.find(obs_type);
    double count = (it != type_counts_.end()) ? static_cast<double>(it->second) : 0.0;
    double share = 1.0 / static_cast<double>(n_types);
    if (weight_total > 0.0) {
      auto wit = default_type_weights_.find(obs_type);
      share = (wit != default_type_weights_.end())
                  ? std::max(0.0, wit->second) / weight_total
                  : 0.0;
    }
    dist[obs_type] =
        (count + alpha * static_cast<double>(n_types) * share) / total_with_prior;
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
