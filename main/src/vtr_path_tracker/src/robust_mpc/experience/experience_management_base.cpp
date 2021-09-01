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
 * \file experience_management_base.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_path_tracker/robust_mpc/experience/experience_management_base.hpp>

namespace vtr {
namespace path_tracker {

void ExperienceManagement::initialize_running_experiences(
    vtr::path_tracker::MpcNominalModel &MpcNominalModel, uint64_t &at_vertex_id,
    uint64_t &to_vertex_id, double &turn_radius) {
  MpcNominalModel.initialize_experience(experience_km2_, ros_clock);
  MpcNominalModel.initialize_experience(experience_km1_, ros_clock);
  MpcNominalModel.initialize_experience(experience_k_, ros_clock);

  experience_km2_.at_vertex_id = at_vertex_id;
  experience_km2_.to_vertex_id = to_vertex_id;
  experience_km2_.path_curvature = turn_radius;
  experience_km1_.at_vertex_id = at_vertex_id;
  experience_km1_.to_vertex_id = to_vertex_id;
  experience_km1_.path_curvature = turn_radius;
  experience_k_.at_vertex_id = at_vertex_id;
  experience_k_.to_vertex_id = to_vertex_id;
  experience_k_.path_curvature = turn_radius;
}

void ExperienceManagement::initializeExperience(
    MpcNominalModel::experience_t &experience) {
  // State
  experience.x_k.x_k = Eigen::VectorXf::Zero(STATE_SIZE);

  // Velocity estimate
  experience.x_k.velocity_km1 = Eigen::VectorXf::Zero(VELOCITY_SIZE);

  // Commands
  experience.x_k.command_k = Eigen::VectorXf::Zero(VELOCITY_SIZE);
  experience.x_k.command_km1 = Eigen::VectorXf::Zero(VELOCITY_SIZE);

  // Tracking error and distance
  experience.x_k.dist_along_path_k = 0;
  experience.x_k.tracking_error_k = Eigen::VectorXf::Zero(STATE_SIZE);
  experience.x_k.tracking_error_km1 = Eigen::VectorXf::Zero(STATE_SIZE);
  experience.x_k.tracking_error_k_interp = Eigen::VectorXf::Zero(STATE_SIZE);

  // Vertex info
  pose_graph::VertexId vid;
  experience.at_vertex_id = vid;
  experience.to_vertex_id = vid;
  experience.T_0_v = tf2::Transform();
  experience.transform_time = ros_clock.now();

  // Variables for GP
  experience.gp_data.x_meas = Eigen::VectorXf::Zero(DIST_DEP_SIZE);
  experience.gp_data.g_x_meas = Eigen::VectorXf::Zero(STATE_SIZE);
}

}  // namespace path_tracker
}  // namespace vtr
