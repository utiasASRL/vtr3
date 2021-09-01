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
 * \file experience_management_base.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <vtr_common/timing/time_utils.hpp>
#include <vtr_path_tracker/robust_mpc/optimization/mpc_nominal_model.hpp>

namespace vtr {
namespace path_tracker {

/** \brief */
class ExperienceManagement {
  friend class RCExperienceManagement;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** \brief Constructor */
  /// \todo: (old) Make refresh_experiences configurable
  explicit ExperienceManagement(const rclcpp::Clock &node_clock)
      : ros_clock(node_clock), refresh_experiences(false) {}

  /** \brief Destructor */
  ~ExperienceManagement() = default;

  /** \brief "Experience" at timestep k */
  MpcNominalModel::experience_t experience_k_;
  /** \brief "Experience" at timestep k - 1 */
  MpcNominalModel::experience_t experience_km1_;
  /** \brief "Experience" at timestep k - 2 */
  MpcNominalModel::experience_t experience_km2_;

  /** Typedef vector of experiences, likely stored at a single vertex **/
  typedef std::vector<MpcNominalModel::experience_t> vertexExperienceVec_t;

  /**
   * @brief ExperienceManagement::InitializeExperiences
   *
   * Initialize fields of the experience that are used to the correct size.
   * This relies on correct definition of STATE_SIZE and VELOCITY_SIZE from
   * ...NominalModel.hpp
   */
  void initializeExperience(MpcNominalModel::experience_t &experience);

  /**
   * @brief ExperienceManagement::initialize_running_experiences
   * @param MpcNominalModel: Used since it has the methods to initialize an
   * experience
   * @param at_vertex_id: The current vertex id
   * @param to_vertex_id: The next vertex id along the path
   * @param turn_radius: the current turn radius along the path
   */
  void initialize_running_experiences(MpcNominalModel &MpcNominalModel,
                                      uint64_t &at_vertex_id,
                                      uint64_t &to_vertex_id,
                                      double &turn_radius);

  /** \brief Handles time in ROS2 */
  rclcpp::Clock ros_clock;

 private:
  /** \brief */
  bool refresh_experiences;
#if 0
  /** \brief
 */
  bool flg_recall_live_data_;
  /** \brief
 */
  int max_num_experiences_per_bin_;
  /** \brief
 */
  int target_model_size_;
#endif
};

}  // namespace path_tracker
}  // namespace vtr
