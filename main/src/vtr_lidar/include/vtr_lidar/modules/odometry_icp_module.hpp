/**
 * \file odometry_icp_module.hpp
 * \brief OdometryICPModule class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <random>

#include <steam.hpp>

#include <vtr_common/timing/stopwatch.hpp>
#include <vtr_lidar/cache.hpp>
#include <vtr_tactic/modules/base_module.hpp>

namespace vtr {

namespace lidar {

/** \brief ICP for odometry. */
class OdometryICPModule : public tactic::BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.odometry_icp";

  /** \brief Config parameters. */
  struct Config : public steam::VanillaGaussNewtonSolver::Params {
    float min_matched_ratio = 0.4;

    // trajectory smoothing
    bool trajectory_smoothing = false;
    bool use_constant_acc = true;
    double lin_acc_std_dev_x = 1.0;
    double lin_acc_std_dev_y = 0.01;
    double lin_acc_std_dev_z = 0.01;
    double ang_acc_std_dev_x = 0.01;
    double ang_acc_std_dev_y = 0.01;
    double ang_acc_std_dev_z = 1.0;

    /// ICP parameters
    // number of threads for nearest neighbor search
    int num_threads = 8;
    // initial alignment config
    size_t initial_max_iter = 100;
    size_t initial_num_samples = 1000;
    float initial_max_pairing_dist = 2.0;
    float initial_max_planar_dist = 0.3;
    // refined stage
    size_t refined_max_iter = 10;  // we use a fixed number of iters for now
    size_t refined_num_samples = 5000;
    float refined_max_pairing_dist = 2.0;
    float refined_max_planar_dist = 0.1;
    // error calculation
    float averaging_num_steps = 5;
    float trans_diff_thresh = 0.01;              // threshold on variation of T
    float rot_diff_thresh = 0.1 * M_PI / 180.0;  // threshold on variation of R
  };

  OdometryICPModule(const std::string &name = static_name)
      : tactic::BaseModule{name}, config_(std::make_shared<Config>()) {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 private:
  void runImpl(tactic::QueryCache &qdata,
               const tactic::Graph::ConstPtr &graph) override;

  void computeTrajectory(
      LidarQueryCache &qdata, const tactic::Graph::ConstPtr &graph,
      const steam::se3::TransformEvaluator::Ptr &T_r_m_eval,
      std::map<unsigned int, steam::StateVariableBase::Ptr> &state_vars,
      const steam::ParallelizedCostTermCollection::Ptr &prior_cost_terms);

  std::shared_ptr<steam::se3::SteamTrajInterface> trajectory_ = nullptr;
  Eigen::Matrix<double, 6, 6> smoothing_factor_information_ =
      Eigen::Matrix<double, 6, 6>::Zero();

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;
};

}  // namespace lidar
}  // namespace vtr