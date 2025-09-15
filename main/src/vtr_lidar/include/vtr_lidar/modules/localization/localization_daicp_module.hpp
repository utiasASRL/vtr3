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

    /// Prior terms
    bool use_pose_prior = false;

    /// ICP parameters
    // number of threads for nearest neighbor search
    int num_threads = 4;
    // initial alignment config
    size_t first_num_steps = 3;
    size_t initial_max_iter = 100;
    float initial_max_pairing_dist = 2.0;
    float initial_max_planar_dist = 0.3;
    // refined stage
    size_t refined_max_iter = 20;  // we use a fixed number of iters for now
    float refined_max_pairing_dist = 2.0;
    float refined_max_planar_dist = 0.1;
    // error calculation
    float averaging_num_steps = 10;
    float trans_diff_thresh = 0.01;              // threshold on variation of T
    float rot_diff_thresh = 0.1 * M_PI / 180.0;  // threshold on variation of R
    // steam optimizer
    bool verbose = false;
    unsigned int max_iterations = 1;

    /// Success criteria
    float min_matched_ratio = 0.4;

    // Time above which the localization ICP module will be skipped. (in ms)
    // For lidar running at 10 Hz, a good choice is < 50 ms to allow both odometry and localization to run
    float target_loc_time = 1000000.0;

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
        void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
                  const tactic::Graph::Ptr &graph,
                  const tactic::TaskExecutor::Ptr &executor) override;
                  
        Config::ConstPtr config_;

        VTR_REGISTER_MODULE_DEC_TYPE(LocalizationDAICPModule);
};

}  // namespace lidar
}  // namespace vtr