// Copyright 2026, Autonomous Space Robotics Lab (ASRL)
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
 * \file gyro_bias_estimation_module.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */

#ifdef VTR_ENABLE_RADAR
#include "vtr_radar/cache.hpp"
#endif

#ifdef VTR_ENABLE_LIDAR
#include "vtr_lidar/cache.hpp"
#endif

#include "vtr_tactic/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace navigation {

class GyroBiasEstimationModule : public tactic::BaseModule {
 public:
  using ImuMsg = sensor_msgs::msg::Imu;

  #if defined(VTR_ENABLE_RADAR)
    using CacheType = radar::RadarQueryCache;
  #elif defined(VTR_ENABLE_LIDAR)
    using CacheType = lidar::LidarQueryCache;
  #else
    using CacheType = tactic::QueryCache;
  #endif

  /** \brief Static module identifier. */
  static constexpr auto static_name = "navigation.gyro_bias_removal";

    /** \brief Config parameters. */
  struct Config : public BaseModule::Config {
    PTR_TYPEDEFS(Config);

    double max_vel = 0.1;  // maximum velocity below which gyro messages will be averaged
    double alpha = 0.001;
    double min_bias_time = 0.0;  // seconds below max_vel required before bias updates start

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  GyroBiasEstimationModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {}

 private:
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;

  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  unsigned long count_ = 0;

  // Raw gyro readings cached before the in-place correction step, for the update step to use.
  std::vector<Eigen::Vector3d> raw_gyro_cache_;

  // Running total of the bias subtracted from the gyro readings, per axis.
  Eigen::Vector3d total_bias_correction_ = Eigen::Vector3d::Zero();

  // Toggles each call since odo_success alone can't tell a not-yet-run odometry step apart from a failed one.
  bool awaiting_odometry_ = true;

  // Stamp at which the velocity most recently dropped below max_vel, or -1 if not currently below it.
  tactic::Timestamp low_vel_start_stamp_ = -1;

  VTR_REGISTER_MODULE_DEC_TYPE(GyroBiasEstimationModule);
};

}  // namespace navigation
}  // namespace vtr