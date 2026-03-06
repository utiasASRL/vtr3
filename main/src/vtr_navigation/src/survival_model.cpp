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
 * \file survival_model.cpp
 * \brief Kaplan-Meier survival model implementation.
 */
#include "vtr_navigation/survival_model.hpp"

#include <algorithm>
#include <fstream>
#include <cmath>
#include <numeric>

#include "yaml-cpp/yaml.h"
#include "vtr_logging/logging.hpp"

namespace vtr {
namespace navigation {

bool SurvivalModel::loadFromFile(const std::string& path) {
  std::lock_guard<std::mutex> lock(mutex_);
  data_file_path_ = path;
  
  try {
    std::ifstream file(path);
    if (!file.good()) {
      CLOG(INFO, "navigation") << "HSHMAT SurvivalModel: No existing data file at " << path;
      return false;
    }
    
    YAML::Node root = YAML::LoadFile(path);
    if (!root || !root.IsMap()) {
      CLOG(WARNING, "navigation") << "HSHMAT SurvivalModel: Invalid YAML format in " << path;
      return false;
    }
    
    samples_.clear();
    estimates_.clear();
    
    for (const auto& kv : root) {
      std::string obs_type = kv.first.as<std::string>();
      const YAML::Node& samples_node = kv.second;
      
      if (!samples_node.IsSequence()) continue;
      
      std::vector<SurvivalSample>& type_samples = samples_[obs_type];
      for (const auto& sample_node : samples_node) {
        double time = sample_node["time"].as<double>();
        bool censored = sample_node["censored"].as<bool>();
        type_samples.emplace_back(time, censored);
      }
      
      CLOG(INFO, "navigation") << "HSHMAT SurvivalModel: Loaded " << type_samples.size()
                               << " samples for '" << obs_type << "'";
    }
    
    // Rebuild estimates for all loaded types
    for (const auto& kv : samples_) {
      rebuildEstimate(kv.first);
    }
    
    CLOG(INFO, "navigation") << "HSHMAT SurvivalModel: Successfully loaded from " << path;
    return true;
    
  } catch (const std::exception& e) {
    CLOG(ERROR, "navigation") << "HSHMAT SurvivalModel: Failed to load from " << path
                              << ": " << e.what();
    return false;
  }
}

bool SurvivalModel::saveToFile(const std::string& path) const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  const std::string& save_path = path.empty() ? data_file_path_ : path;
  if (save_path.empty()) {
    CLOG(WARNING, "navigation") << "HSHMAT SurvivalModel: No path specified for save";
    return false;
  }
  
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    
    for (const auto& kv : samples_) {
      out << YAML::Key << kv.first;
      out << YAML::Value << YAML::BeginSeq;
      
      for (const auto& sample : kv.second) {
        out << YAML::BeginMap;
        out << YAML::Key << "time" << YAML::Value << sample.time;
        out << YAML::Key << "censored" << YAML::Value << sample.censored;
        out << YAML::EndMap;
      }
      
      out << YAML::EndSeq;
    }
    
    out << YAML::EndMap;
    
    std::ofstream file(save_path);
    if (!file.good()) {
      CLOG(ERROR, "navigation") << "HSHMAT SurvivalModel: Cannot write to " << save_path;
      return false;
    }
    
    file << out.c_str();
    CLOG(INFO, "navigation") << "HSHMAT SurvivalModel: Saved to " << save_path;
    return true;
    
  } catch (const std::exception& e) {
    CLOG(ERROR, "navigation") << "HSHMAT SurvivalModel: Failed to save to " << save_path
                              << ": " << e.what();
    return false;
  }
}

void SurvivalModel::addSample(const std::string& obs_type, double duration, bool censored) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  samples_[obs_type].emplace_back(duration, censored);
  rebuildEstimate(obs_type);
  
  CLOG(INFO, "navigation") << "HSHMAT SurvivalModel: Added sample for '" << obs_type
                           << "': duration=" << duration << "s, censored=" << censored
                           << " (total samples: " << samples_[obs_type].size() << ")";
  
  // Auto-save after each sample
  if (!data_file_path_.empty()) {
    // Release lock before calling saveToFile (which acquires its own lock)
    // Actually, saveToFile uses const and acquires lock, so we need to save without lock
    // For simplicity, save inline here
    try {
      YAML::Emitter out;
      out << YAML::BeginMap;
      for (const auto& kv : samples_) {
        out << YAML::Key << kv.first;
        out << YAML::Value << YAML::BeginSeq;
        for (const auto& sample : kv.second) {
          out << YAML::BeginMap;
          out << YAML::Key << "time" << YAML::Value << sample.time;
          out << YAML::Key << "censored" << YAML::Value << sample.censored;
          out << YAML::EndMap;
        }
        out << YAML::EndSeq;
      }
      out << YAML::EndMap;
      std::ofstream file(data_file_path_);
      if (file.good()) {
        file << out.c_str();
      }
    } catch (...) {}
  }
}

void SurvivalModel::addSeedSamples(const std::string& obs_type, const std::vector<double>& durations) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  for (double d : durations) {
    samples_[obs_type].emplace_back(d, false);  // Seed samples are uncensored
  }
  rebuildEstimate(obs_type);
  
  CLOG(INFO, "navigation") << "HSHMAT SurvivalModel: Added " << durations.size()
                           << " seed samples for '" << obs_type << "'";
}

void SurvivalModel::rebuildEstimate(const std::string& obs_type) {
  // Must be called with lock held
  auto it = samples_.find(obs_type);
  if (it == samples_.end() || it->second.empty()) {
    estimates_.erase(obs_type);
    return;
  }
  
  const auto& type_samples = it->second;
  KMEstimate& est = estimates_[obs_type];
  est.times.clear();
  est.survival.clear();
  est.max_event_time = 0.0;
  
  // Collect all unique times and sort
  std::vector<std::pair<double, bool>> events;  // (time, censored)
  for (const auto& s : type_samples) {
    events.emplace_back(s.time, s.censored);
    if (!s.censored && s.time > est.max_event_time) {
      est.max_event_time = s.time;
    }
  }
  std::sort(events.begin(), events.end());
  
  // Kaplan-Meier product-limit estimator
  // S(t) = product over t_i <= t of (1 - d_i / n_i)
  // where d_i = events at t_i, n_i = at risk just before t_i
  
  int n = static_cast<int>(events.size());
  double S = 1.0;
  
  est.times.push_back(0.0);
  est.survival.push_back(1.0);
  
  int i = 0;
  while (i < n) {
    double t = events[i].first;
    int d = 0;  // deaths (uncensored events) at this time
    int c = 0;  // censored at this time
    
    // Count all events at this time point
    while (i < n && events[i].first == t) {
      if (events[i].second) {
        c++;
      } else {
        d++;
      }
      i++;
    }
    
    // n_i = number at risk = total remaining (those with time >= t)
    int n_i = n - (i - d - c);
    
    if (n_i > 0 && d > 0) {
      S *= (1.0 - static_cast<double>(d) / static_cast<double>(n_i));
    }
    
    est.times.push_back(t);
    est.survival.push_back(S);
  }
}

double SurvivalModel::survival(const std::string& obs_type, double t) const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  auto it = estimates_.find(obs_type);
  if (it == estimates_.end() || it->second.times.empty()) {
    // No data - assume obstacle never clears (conservative)
    return 1.0;
  }
  
  const KMEstimate& est = it->second;
  
  // Step function: find largest time <= t
  auto pos = std::upper_bound(est.times.begin(), est.times.end(), t);
  if (pos == est.times.begin()) {
    return 1.0;
  }
  --pos;
  size_t idx = std::distance(est.times.begin(), pos);
  return est.survival[idx];
}

double SurvivalModel::conditionalSurvival(const std::string& obs_type, double t, double elapsed) const {
  // S(t | T > elapsed) = S(t + elapsed) / S(elapsed)
  double S_elapsed = survival(obs_type, elapsed);
  if (S_elapsed <= 0.0) {
    return 0.0;  // Already past all observed events
  }
  double S_t_plus_elapsed = survival(obs_type, t + elapsed);
  return S_t_plus_elapsed / S_elapsed;
}

double SurvivalModel::meanSurvivalTime(const std::string& obs_type, double W_max) const {
  // E[T] = integral from 0 to W_max of S(t) dt
  // Use trapezoidal rule
  const int steps = 100;
  double dt = W_max / steps;
  double integral = 0.0;
  
  double S_prev = survival(obs_type, 0.0);
  for (int i = 1; i <= steps; ++i) {
    double t = i * dt;
    double S_curr = survival(obs_type, t);
    integral += 0.5 * (S_prev + S_curr) * dt;
    S_prev = S_curr;
  }
  
  return integral;
}

double SurvivalModel::conditionalExpectedTime(const std::string& obs_type, double elapsed, double W_max) const {
  // E[T | T > elapsed] = elapsed + integral from elapsed to W_max of S(t | T > elapsed) dt
  // where S(t | T > elapsed) = S(t) / S(elapsed)
  //
  // E[T | T > elapsed] = elapsed + integral from elapsed to W_max of (S(t) / S(elapsed)) dt
  //                    = elapsed + (1/S(elapsed)) * integral from elapsed to W_max of S(t) dt
  
  double S_elapsed = survival(obs_type, elapsed);
  if (S_elapsed <= 1e-15) {
    // All observations cleared before elapsed - return elapsed as best estimate
    return elapsed;
  }
  
  // Integrate S(t) from elapsed to W_max
  const int steps = 100;
  double range = W_max - elapsed;
  if (range <= 0) {
    return elapsed;
  }
  double dt = range / steps;
  double integral = 0.0;
  
  double S_prev = survival(obs_type, elapsed);
  for (int i = 1; i <= steps; ++i) {
    double t = elapsed + i * dt;
    double S_curr = survival(obs_type, t);
    integral += 0.5 * (S_prev + S_curr) * dt;
    S_prev = S_curr;
  }
  
  return elapsed + integral / S_elapsed;
}

double SurvivalModel::maxObservedEventTime(const std::string& obs_type) const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  auto it = estimates_.find(obs_type);
  if (it == estimates_.end()) {
    return 0.0;
  }
  return it->second.max_event_time;
}

std::vector<SurvivalSample> SurvivalModel::getSamples(const std::string& obs_type) const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  auto it = samples_.find(obs_type);
  if (it == samples_.end()) {
    return {};
  }
  return it->second;
}

size_t SurvivalModel::sampleCount(const std::string& obs_type) const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  auto it = samples_.find(obs_type);
  if (it == samples_.end()) {
    return 0;
  }
  return it->second.size();
}

bool SurvivalModel::hasData(const std::string& obs_type) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_.find(obs_type) != samples_.end() && !samples_.at(obs_type).empty();
}

void SurvivalModel::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.clear();
  estimates_.clear();
}

}  // namespace navigation
}  // namespace vtr
