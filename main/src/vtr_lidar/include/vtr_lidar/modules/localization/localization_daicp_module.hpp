#pragma once

#include "steam.hpp"

#include "vtr_lidar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {

namespace lidar {

/** \brief ICP for localization. */
class LocalizationDAICPModule : public tactic::BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.localization_daicp";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    /// general
    int num_threads = 4;                        // number of threads for nearest neighbor search
    float target_loc_time = 1000000.0;          // time above which the localization ICP module will be skipped. (in ms)
                                                // For lidar running at 10 Hz, a good choice is < 50 ms to allow both odometry and localization to run
    // iteration control
    size_t first_num_steps = 3;
    size_t initial_max_iter = 100;
    // data association
    float lambda = 1.0;                         // weight for spatial vs curvature in correspondence search
    float curvature_similarity_thresh = 0.5;    // threshold on curvature difference for correspondence
    float initial_max_pairing_dist = 2.0;
    float initial_max_planar_dist = 0.3;
    // daicp
    int max_gn_iter = 2;                        // max gauss-newton iterations per daicp step
    // degeneracy threshold
    double degeneracy_thresh = 100.0;           // use relative condition number to set eigenvalue threshold
    // daicp - range and bearing noise model
    double sigma_d = 0.02;                      // range noise std (2cm)
    double sigma_az = M_PI / 180.0 * 0.03;      // azimuth noise std (0.03 degrees)
    double sigma_el = M_PI / 180.0 * 0.03;      // elevation noise std (0.03 degrees)
    // daicp - convergence checks (numbers taken from STEAM's ICP implementation)
    double abs_cost_thresh = 1e-12;             // absolute cost threshold for convergence
    double abs_cost_change_thresh = 1e-8;       // absolute parameter change threshold for convergence
    double rel_cost_change_thresh = 1e-8;       // relative cost change threshold for convergence
    double zero_gradient_thresh = 1e-8;         // zero gradient threshold for convergence
    double inner_tolerance = 1e-6;              // tolerance for gauss-newton solver
    // refinement
    size_t refined_max_iter = 20;               // we use a fixed number of iters for now
    float refined_max_pairing_dist = 2.0;
    float refined_max_planar_dist = 0.1;
    // error calculation
    float averaging_num_steps = 10;
    float trans_diff_thresh = 0.01;             // threshold on variation of T
    float rot_diff_thresh = 0.1 * M_PI / 180.0; // threshold on variation of R
    // outlier rejection
    float trans_outlier_thresh = 0.1;           // threshold on translation outlier rejection to default to odometry
    float rot_outlier_thresh = 0.001;           // threshold on rotation outlier rejection to default to odometry
    float min_matched_ratio = 0.4;              // success criteria
    // online gyroscope bias
    bool calc_gy_bias = false;
    float calc_gy_bias_thresh = 1.0;

    // handcrafted degeneracy threshold
    // double eigenvalue_threshold = 100.0;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  LocalizationDAICPModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule(module_factory, name), config_(config) {}

 private:

  int64_t timestamp_prev_ = 0;
  Eigen::Matrix4d T_r_v_loc_prev_ = Eigen::Matrix4d::Identity();
  
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;
            
  Config::ConstPtr config_;

  VTR_REGISTER_MODULE_DEC_TYPE(LocalizationDAICPModule);
};

}  // namespace lidar
}  // namespace vtr