
/*
File:           path_tracker_mpc_implementation.hpp
Edited By:      Chris Ostafew
Date:           Aug 11, 2014

Purpose:         To Do

Functions:      To Do
*/

#include <vtr/path_tracker/robust_mpc/experience/experience_management_base.h>


namespace vtr {
namespace path_tracker {

// TODO: Make refresh_experiences configurable
ExperienceManagement::ExperienceManagement() :
  refresh_experiences(false) {
}

ExperienceManagement::~ExperienceManagement(){}

void ExperienceManagement::set_params(bool flg_recall_live_data, int max_num_experiences_per_bin, int target_model_size){
  flg_recall_live_data_ = flg_recall_live_data;
  max_num_experiences_per_bin_ = max_num_experiences_per_bin;
  target_model_size_ = target_model_size;

}


void ExperienceManagement::initialize_running_experiences(vtr::path_tracker::MpcNominalModel & MpcNominalModel,
                                                          boost::uint64_t & at_vertex_id,
                                                          boost::uint64_t & to_vertex_id,
                                                          double & turn_radius){
  MpcNominalModel.initialize_experience(experience_km2_);
  MpcNominalModel.initialize_experience(experience_km1_);
  MpcNominalModel.initialize_experience(experience_k_);

  //asrl::path_tracker::MpcNominalModel::initialize_experience(prevState_);
  experience_km2_.at_vertex_id = at_vertex_id;
  experience_km2_.to_vertex_id = to_vertex_id;
  experience_km2_.path_curvature = turn_radius;
  experience_km1_.at_vertex_id = at_vertex_id;
  experience_km1_.to_vertex_id = to_vertex_id;
  experience_km1_.path_curvature = turn_radius;
  experience_k_.at_vertex_id = at_vertex_id;
  experience_k_.to_vertex_id = to_vertex_id;
  experience_k_.path_curvature = turn_radius;

  return;
}



void ExperienceManagement::initializeExperience(MpcNominalModel::experience_t& experience)
{
  // State
  experience.x_k.x_k = Eigen::VectorXf::Zero(STATE_SIZE);

  // Velocity estimate
  experience.x_k.velocity_km1 = Eigen::VectorXf::Zero(VELOCITY_SIZE);

  // Commands
  experience.x_k.command_k   = Eigen::VectorXf::Zero(VELOCITY_SIZE);
  experience.x_k.command_km1 = Eigen::VectorXf::Zero(VELOCITY_SIZE);

  // Tracking error and distance
  experience.x_k.dist_along_path_k       = 0;
  experience.x_k.tracking_error_k        = Eigen::VectorXf::Zero(STATE_SIZE);
  experience.x_k.tracking_error_km1      = Eigen::VectorXf::Zero(STATE_SIZE);
  experience.x_k.tracking_error_k_interp = Eigen::VectorXf::Zero(STATE_SIZE);

  // Vertex info
  asrl::pose_graph::VertexId vid;
  experience.at_vertex_id = vid;
  experience.to_vertex_id = vid;
  experience.T_0_v = tf::Transform();
  experience.transform_time = ros::Time::now();

  // Variables for GP
  experience.gp_data.x_meas = Eigen::VectorXf::Zero(DIST_DEP_SIZE);
  experience.gp_data.g_x_meas = Eigen::VectorXf::Zero(STATE_SIZE);

  return;
}



}} // end vtr::path_tracker
