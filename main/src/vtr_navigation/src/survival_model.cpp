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
        int episode = sample_node["episode"] ? sample_node["episode"].as<int>() : 0;
        type_samples.emplace_back(time, censored, episode);
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
        out << YAML::Key << "episode" << YAML::Value << sample.episode;
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

void SurvivalModel::addSample(const std::string& obs_type, double duration, bool censored, int episode) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  samples_[obs_type].emplace_back(duration, censored, episode);
  rebuildEstimate(obs_type);
  
  CLOG(INFO, "navigation") << "HSHMAT SurvivalModel: Added sample for '" << obs_type
                           << "': duration=" << duration << "s, censored=" << censored
                           << ", episode=" << episode
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
          out << YAML::Key << "episode" << YAML::Value << sample.episode;
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
  est.uncensored_times.clear();
  est.survival_at_uncensored.clear();
  est.max_event_time = 0.0;
  est.tail_lambda = 0.0;
  
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
      // Store uncensored event times and their survival values (matches sim's event_times)
      est.uncensored_times.push_back(t);
      est.survival_at_uncensored.push_back(S);
    }
    
    est.times.push_back(t);
    est.survival.push_back(S);
  }
  
  // Compute exponential tail hazard rate (matches sim's km.py logic)
  // Local hazard over last few uncensored events: lambda = -ln(S_last/S_anchor) / (t_last - t_anchor)
  double t_last = est.max_event_time;
  double s_last = S;
  
  if (t_last > 0.0 && s_last > 0.0 && s_last < 1.0) {
    const int LOCAL_K = 5;
    int n_ev = static_cast<int>(est.uncensored_times.size());
    if (n_ev >= 2) {
      int anchor_idx = std::max(0, n_ev - 1 - LOCAL_K);
      double t_anchor = est.uncensored_times[anchor_idx];
      double s_anchor = est.survival_at_uncensored[anchor_idx];
      double dt = t_last - t_anchor;
      if (dt > 0.0 && s_last < s_anchor && s_last > 0.0) {
        est.tail_lambda = -std::log(s_last / s_anchor) / dt;
      } else {
        est.tail_lambda = -std::log(s_last) / t_last;
      }
    } else {
      est.tail_lambda = -std::log(s_last) / t_last;
    }
  }
}

double SurvivalModel::survival(const std::string& obs_type, double t) const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  auto it = estimates_.find(obs_type);
  if (it == estimates_.end() || it->second.times.empty()) {
    // No data - assume obstacle clears immediately (promotes exploration)
    // With S(t)=0, expected wait is 0, so robot will try this path
    // rather than avoiding it conservatively
    return 0.0;
  }
  
  const KMEstimate& est = it->second;
  
  // Check if past the last uncensored event - use exponential tail
  double t_last = est.max_event_time;
  if (t > t_last && t_last > 0.0) {
    double s_last = est.survival.back();
    if (est.tail_lambda > 0.0) {
      // Exponential tail: S(t) = s_last * exp(-lambda * (t - t_last))
      return s_last * std::exp(-est.tail_lambda * (t - t_last));
    } else {
      // No slope estimate: plateau at s_last
      return s_last;
    }
  }
  
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
  // Matches sim's km.py: exact step integral up to t_last, then exponential tail to W_max
  std::lock_guard<std::mutex> lock(mutex_);
  
  auto it = estimates_.find(obs_type);
  if (it == estimates_.end() || it->second.times.empty()) {
    return 60.0;  // Default fallback (same as sim)
  }
  
  const KMEstimate& est = it->second;
  double t_last = est.max_event_time;
  double s_last = est.survival.back();
  
  // Exact step function integral up to t_last
  double result = 0.0;
  for (size_t i = 1; i < est.times.size(); ++i) {
    double width = est.times[i] - est.times[i-1];
    double height = est.survival[i-1];  // Right-continuous: use left value
    result += width * height;
  }
  
  // Exponential tail integral from t_last to W_max (matches sim's km.py)
  if (W_max > t_last && s_last > 0.0) {
    double lam = est.tail_lambda;
    if (lam > 0.0) {
      // S_tail(t) = s_last * exp(-lam * (t - t_last))
      // Integral = s_last * (1 - exp(-lam * (W_max - t_last))) / lam
      result += s_last * (1.0 - std::exp(-lam * (W_max - t_last))) / lam;
    } else {
      // Degenerate: rectangle at s_last up to W_max
      result += s_last * (W_max - t_last);
    }
  }
  
  return result;
}

double SurvivalModel::conditionalExpectedTime(const std::string& obs_type, double c, double W_max) const {
  // E[T | T > c] = c + integral_{c}^{W_max} S(t)/S(c) dt
  //
  // FIFO fix: Use exact analytical integration over KM steps + exponential tail
  // to guarantee E[T | T > c] is non-decreasing in c.
  // This matches sim's km.py conditional_expected_km() exactly.
  
  std::lock_guard<std::mutex> lock(mutex_);
  
  auto it = estimates_.find(obs_type);
  if (it == estimates_.end() || it->second.uncensored_times.empty()) {
    // No data: return c (can't estimate) - matches sim
    return c;
  }
  
  const KMEstimate& est = it->second;
  const auto& times = est.uncensored_times;        // Only uncensored event times (like sim's event_times)
  const auto& surv = est.survival_at_uncensored;   // Survival at those times (like sim's survival_at_events)
  
  // Compute S(c) using the full step function (handles all times correctly)
  double S_c = 1.0;
  double t_last = est.max_event_time;
  double s_last = surv.empty() ? 1.0 : surv.back();
  double lam = est.tail_lambda;
  
  if (c > t_last && t_last > 0.0) {
    // In exponential tail region
    if (lam > 0.0) {
      S_c = s_last * std::exp(-lam * (c - t_last));
    } else {
      S_c = s_last;
    }
  } else {
    // In step function region - use uncensored times for lookup
    auto pos = std::upper_bound(times.begin(), times.end(), c);
    if (pos == times.begin()) {
      S_c = 1.0;  // Before first event
    } else {
      --pos;
      size_t idx = std::distance(times.begin(), pos);
      S_c = surv[idx];
    }
  }
  
  if (S_c <= 1e-15) {
    return c;
  }
  
  // Exact integration of S(t) from c to W_max
  // Matches sim's conditional_expected_km logic exactly:
  //   1. Sum over KM steps from c to t_last
  //   2. Add exponential tail from max(c, t_last) to W_max
  
  double integral = 0.0;
  
  // Find event times > c (mask in sim)
  auto first_after = std::upper_bound(times.begin(), times.end(), c);
  
  if (first_after != times.end()) {
    // Sum over KM steps from c to t_last
    // Build interval endpoints: [c, t_1, t_2, ..., t_last]
    // Survival just after c (handles case where c is exactly at an event)
    size_t start_idx = std::distance(times.begin(), first_after);
    
    // S_after_c: survival just after c (right-continuous)
    // If c < first event time, S = 1.0
    // Otherwise, S = survival at the event just before or at c
    double S_after_c;
    if (start_idx == 0) {
      S_after_c = 1.0;  // c is before first uncensored event
    } else {
      S_after_c = surv[start_idx - 1];  // survival at event just before c
    }
    
    // Integrate: sum of width * S(left endpoint)
    // [c, first_after_time) uses S_after_c
    // [first_after_time, next_time) uses surv[first_after_idx], etc.
    double prev_t = c;
    double prev_S = S_after_c;
    
    for (size_t i = start_idx; i < times.size(); ++i) {
      double t_i = times[i];
      double width = t_i - prev_t;
      if (width > 0) {
        integral += width * prev_S;
      }
      prev_t = t_i;
      prev_S = surv[i];
    }
    // Note: we've now integrated up to t_last (times.back() == t_last == max_event_time)
  }
  // else: c >= t_last: no KM steps left, just tail contribution
  
  // Add exponential tail contribution past t_last
  // Integral from max(c, t_last) to W_max of S(t) dt
  double tail_start = std::max(c, t_last);
  
  if (lam > 1e-15 && W_max > tail_start) {
    // Exponential tail: S(t) = s_last * exp(-lam * (t - t_last)) for t > t_last
    // Integral from tail_start to W_max:
    // = s_last * exp(-lam * (tail_start - t_last)) / lam * [1 - exp(-lam * (W_max - tail_start))]
    double S_at_tail_start = s_last * std::exp(-lam * (tail_start - t_last));
    double tail_integral = S_at_tail_start / lam * (1.0 - std::exp(-lam * (W_max - tail_start)));
    integral += tail_integral;
  } else if (W_max > tail_start && s_last > 1e-15) {
    // No exponential tail or lam=0: constant survival s_last until W_max
    integral += s_last * (W_max - tail_start);
  }
  // else: no contribution (survival is 0 or W_max <= tail_start)
  
  return c + integral / S_c;
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
