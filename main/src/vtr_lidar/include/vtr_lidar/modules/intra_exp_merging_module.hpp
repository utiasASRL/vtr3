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
 * \file inter_exp_merging_module.hpp
 * \brief IntraExpMergingModule class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_tactic/modules/base_module.hpp>

// visualization
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace vtr {
namespace lidar {

/**
 * \brief Uses ray-tracing to detect short-term dynamic objects.
 * Asynchronous. Optional.
 */
class IntraExpMergingModule : public tactic::BaseModule {
 public:
  using Ptr = std::shared_ptr<IntraExpMergingModule>;
  using WeakPtr = std::weak_ptr<IntraExpMergingModule>;
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  static constexpr auto static_name = "lidar.intra_exp_merging";

  /** \brief Collection of config parameters */
  struct Config {
    int depth = 0;

    bool visualize = false;
  };

  /** \brief The task to be executed. */
  class Task : public tactic::BaseTask {
   public:
    Task(const IntraExpMergingModule::Ptr &module,
         const std::shared_ptr<const Config> &config,
         const tactic::VertexId &target_vid, const unsigned &priority = 0)
        : tactic::BaseTask(priority),
          module_(module),
          config_(config),
          target_vid_(target_vid) {}

    void run(const tactic::AsyncTaskExecutor::Ptr &executor,
             const tactic::Graph::Ptr &graph) override;

   private:
    IntraExpMergingModule::WeakPtr module_;

    std::shared_ptr<const Config> config_;

    const tactic::VertexId target_vid_;
  };

  IntraExpMergingModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()) {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

  std::shared_ptr<const Config> config() const { return config_; }

  rclcpp::Publisher<PointCloudMsg>::SharedPtr &oldMapPublisher() {
    return old_map_pub_;
  }
  rclcpp::Publisher<PointCloudMsg>::SharedPtr &newMapPublisher() {
    return new_map_pub_;
  }

 private:
  void runImpl(tactic::QueryCache &, const tactic::Graph::ConstPtr &) override;

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;  /// \todo no need to be a shared pointer.

  /** \brief mutex to make publisher thread safe */
  std::mutex mutex_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr old_map_pub_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr new_map_pub_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr scan_pub_;
};

}  // namespace lidar
}  // namespace vtr