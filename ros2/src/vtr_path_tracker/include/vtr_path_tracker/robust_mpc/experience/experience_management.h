#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>

#include <vtr_path_tracker/robust_mpc/optimization/mpc_nominal_model.h>
#include <vtr_path_tracker/base.h>
#include <vtr_path_tracker/robust_mpc/mpc/mpc_types.h>

#include <vtr_common/rosutils/transformations.hpp>
#include <vtr_common/timing/time_utils.hpp>

#include <vtr_pose_graph/index/rc_graph/rc_run.hpp>

#include <vtr_messages/msg/experience.hpp>
#include <vtr_path_tracker/robust_mpc/experience/experience_management_base.h>

namespace vtr {
namespace path_tracker {

using Duration = common::timing::duration_ms;

using RosExperience = vtr_messages::msg::Experience;

class RCExperienceManagement : public ExperienceManagement {
  friend class ExperienceManagement;

 protected:

  struct ResultStream {
    static constexpr auto status = "control_status";
    static constexpr auto experience = "control_experience";
    static constexpr auto prediction = "control_prediction";
  };

  MpcNominalModel nominal_model_;
  std::shared_ptr<Graph> graph_;

 public:

  /**
   * @brief RCExperienceManagement::RCExperienceManagement Initialize the graph and call the constructor to the old ExperienceManagement
   * @param graph: shared pointer to the graph
   */
  explicit RCExperienceManagement(const std::shared_ptr<Graph> &graph, rclcpp::Clock node_clock);

  /**
   * @brief Wrapper around MpcNominalModel::compute_disturbance_for_experience_km2
   *
   * Computing for km2 because we only know velocity for km1 at time k, which
   * is the velocity that should be compared to the commanded velocity at
   * time km2
   * @return
   */
  bool computeDisturbancesForExperienceKm2();

  /**
   * @brief RCExperienceManagement::computeVelocitiesForExperienceKm1
   * Set the velocity for experience_km1 using the state at the current
   * time-step and the state at the previous step, which are stored
   * in experience_k and experience_km1.
   */
  void computeVelocitiesForExperienceKm1();

  /**
   * @brief RCExperienceManagement::computeVelocitiesFromState
   * use finite difference to compute velocity between state_km1 and state_k
   * @param velocity: output is [v, omega]^T
   * @param state_km1: previous state
   * @param state_k: current state
   * @param d_t: time difference
   */
  void computeVelocitiesFromState(Eigen::VectorXf &velocity,
                                  const Eigen::VectorXf &state_km1,
                                  const Eigen::VectorXf &state_k,
                                  const float d_t);

};

} // path_tracker
} // vtr
