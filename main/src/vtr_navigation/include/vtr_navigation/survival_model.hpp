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
 * \file survival_model.hpp
 * \brief Kaplan-Meier survival model for obstacle wait time estimation.
 * 
 * HSHMAT: This implements the Kaplan-Meier estimator for modeling obstacle
 * clearance times. The model persists across runs, accumulating data to
 * improve wait time predictions over time.
 */
#pragma once

#include <string>
#include <vector>
#include <map>
#include <mutex>

namespace vtr {
namespace navigation {

/**
 * \brief A single survival sample (observation of obstacle duration).
 * 
 * - time: Duration in seconds the obstacle was observed
 * - censored: true if we rerouted before obstacle cleared (right-censored),
 *             false if we observed the actual clearance time (uncensored)
 */
struct SurvivalSample {
  double time;
  bool censored;
  
  SurvivalSample(double t = 0.0, bool c = false) : time(t), censored(c) {}
};

/**
 * \brief Kaplan-Meier survival model for obstacle clearance times.
 * 
 * Maintains per-obstacle-type survival data and computes:
 * - S(t): Probability that obstacle is still present at time t
 * - Mean survival time
 * - Conditional survival S(t | T > c)
 * 
 * Data persists to a YAML file so the model improves across runs.
 */
class SurvivalModel {
 public:
  SurvivalModel() = default;
  
  /**
   * \brief Load samples from a YAML file.
   * \param path Path to the survival data file
   * \return true if loaded successfully, false otherwise
   */
  bool loadFromFile(const std::string& path);
  
  /**
   * \brief Save samples to a YAML file.
   * \param path Path to the survival data file
   * \return true if saved successfully, false otherwise
   */
  bool saveToFile(const std::string& path) const;
  
  /**
   * \brief Add a new sample for an obstacle type.
   * \param obs_type Obstacle type (e.g., "person", "chair")
   * \param duration Duration in seconds
   * \param censored true if rerouted before clear, false if observed clear
   */
  void addSample(const std::string& obs_type, double duration, bool censored);
  
  /**
   * \brief Add seed samples for initialization (uncensored).
   * \param obs_type Obstacle type
   * \param durations Vector of known clearance durations
   */
  void addSeedSamples(const std::string& obs_type, const std::vector<double>& durations);
  
  /**
   * \brief Compute survival probability S(t) for an obstacle type.
   * \param obs_type Obstacle type
   * \param t Time in seconds
   * \return Probability that obstacle is still present at time t
   */
  double survival(const std::string& obs_type, double t) const;
  
  /**
   * \brief Compute conditional survival S(t | T > elapsed).
   * \param obs_type Obstacle type
   * \param t Time from now
   * \param elapsed Time already waited
   * \return Conditional probability
   */
  double conditionalSurvival(const std::string& obs_type, double t, double elapsed) const;
  
  /**
   * \brief Compute mean survival time for an obstacle type.
   * \param obs_type Obstacle type
   * \param W_max Maximum time to integrate to
   * \return Expected time until obstacle clears
   */
  double meanSurvivalTime(const std::string& obs_type, double W_max) const;
  
  /**
   * \brief Get the maximum observed event time for an obstacle type.
   * \param obs_type Obstacle type
   * \return Maximum uncensored time, or 0 if no uncensored samples
   */
  double maxObservedEventTime(const std::string& obs_type) const;
  
  /**
   * \brief Get all samples for an obstacle type.
   */
  std::vector<SurvivalSample> getSamples(const std::string& obs_type) const;
  
  /**
   * \brief Get count of samples for an obstacle type.
   */
  size_t sampleCount(const std::string& obs_type) const;
  
  /**
   * \brief Check if we have any data for an obstacle type.
   */
  bool hasData(const std::string& obs_type) const;
  
  /**
   * \brief Clear all data (for testing).
   */
  void clear();

 private:
  /**
   * \brief Internal KM computation structure.
   * Precomputed from samples for efficient S(t) queries.
   */
  struct KMEstimate {
    std::vector<double> times;      // Sorted event times
    std::vector<double> survival;   // S(t) at each time point
    double max_event_time = 0.0;    // Largest uncensored time
  };
  
  /**
   * \brief Rebuild KM estimate from samples.
   */
  void rebuildEstimate(const std::string& obs_type);
  
  mutable std::mutex mutex_;
  std::map<std::string, std::vector<SurvivalSample>> samples_;
  std::map<std::string, KMEstimate> estimates_;
  std::string data_file_path_;
};

}  // namespace navigation
}  // namespace vtr
