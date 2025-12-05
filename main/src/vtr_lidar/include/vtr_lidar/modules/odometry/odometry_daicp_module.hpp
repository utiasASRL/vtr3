
#pragma once

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "steam.hpp"

#include "vtr_lidar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {

namespace lidar {

/** \brief ICP for odometry. */
class OdometryDAICPModule : public tactic::BaseModule {
 public:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.odometry_daicp";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    // continuous-time estimation
    bool use_trajectory_estimation = false;
    int traj_num_extra_states = 0;
    Eigen::Matrix<double, 6, 1> traj_qc_diag =
        Eigen::Matrix<double, 6, 1>::Ones();

    /// ICP parameters
    // number of threads for nearest neighbor search
    int num_threads = 4;
    // initial alignment config
    size_t first_num_steps = 3;
    size_t initial_max_iter = 100;
    float initial_max_pairing_dist = 2.0;
    float initial_max_planar_dist = 0.3;
    // refined stage
    size_t refined_max_iter = 10;  // we use a fixed number of iters for now
    float refined_max_pairing_dist = 2.0;
    float refined_max_planar_dist = 0.1;
    // error calculation
    float averaging_num_steps = 5;
    float trans_diff_thresh = 0.01;              // threshold on variation of T
    float rot_diff_thresh = 0.1 * M_PI / 180.0;  // threshold on variation of R
    // steam optimizer
    bool verbose = false;
    unsigned int max_iterations = 1;

    // gyro weight
    double gyro_cov = 1e-3;
    bool remove_orientation = false;

    // daicp parameters
    int max_gn_iter = 10;
    // daicp - degeneracy threshold
    double degeneracy_thresh = 1e6;
    // daicp - range and bearing noise model
    double sigma_d = 0.02;
    double sigma_az = 0.03 * M_PI / 180.0;
    double sigma_el = 0.03 * M_PI / 180.0;
    // daicp - convergence checks
    double abs_cost_thresh = 1e-4;
    double abs_cost_change_thresh = 1e-4;
    double rel_cost_change_thresh = 1e-4;
    double zero_gradient_thresh = 1e-4;
    double inner_tolerance = 1e-4;
    // prior fusion
    int max_pfusion_iter = 10;
    bool use_L2_loss = true;
    double robust_loss = 0.1;

    /// Success criteria
    float min_matched_ratio = 0.4;
    float max_trans_vel_diff = 1000.0; // m/s
    float max_rot_vel_diff = 1000.0; // m/s
    float max_trans_diff = 1000.0; // m
    float max_rot_diff = 1000.0; // rad

    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  OdometryDAICPModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule(module_factory, name), config_(config) {}

 private:
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;

  VTR_REGISTER_MODULE_DEC_TYPE(OdometryDAICPModule);
};

}  // namespace lidar
}  // namespace vtr