
#include <vtr_path_tracker/robust_mpc/experience/experience_management_base.h>

namespace vtr {
namespace path_tracker {

/// \todo: (old) Make refresh_experiences configurable
ExperienceManagement::ExperienceManagement(const rclcpp::Clock& node_clock) :
    ros_clock(node_clock), refresh_experiences(false) {
}

ExperienceManagement::~ExperienceManagement() = default;

void ExperienceManagement::initialize_running_experiences(vtr::path_tracker::MpcNominalModel &MpcNominalModel,
                                                          uint64_t &at_vertex_id,
                                                          uint64_t &to_vertex_id,
                                                          double &turn_radius) {
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

void ExperienceManagement::initializeExperience(MpcNominalModel::experience_t &experience) {
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

} // path_tracker
} // vtr

