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
 * \file windowed_map_recall_module.hpp
 * \brief WindowedMapRecallModule class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <pcl_conversions/pcl_conversions.h>

#include <vtr_lidar/cache.hpp>
#include <vtr_pose_graph/path/pose_cache.hpp>
#include <vtr_tactic/modules/base_module.hpp>

// visualization
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vtr_messages/msg/point_map.hpp>

namespace vtr {
namespace lidar {

using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using PointXYZMsg = vtr_messages::msg::PointXYZ;
using PointMapMsg = vtr_messages::msg::PointMap;

/** \brief Preprocess raw pointcloud points and compute normals */
class WindowedMapRecallModule : public tactic::BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.windowed_map_recall";

  /** \brief Config parameters */
  struct Config {
    float single_exp_map_voxel_size = 0.1;
    float multi_exp_map_voxel_size = 0.3;
    // short term
    bool remove_short_term_dynamic = false;
    int short_term_min_num_observations = 1;
    float short_term_min_movability = 0.5;
    // long term
    int depth = 1;
    int num_additional_exps = 0;
    int long_term_min_num_observations = 1;
    bool visualize = false;
  };

  WindowedMapRecallModule(const std::string &name = static_name)
      : tactic::BaseModule{name}, config_(std::make_shared<Config>()) {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 private:
  void runImpl(tactic::QueryCache &qdata,
               const tactic::Graph::ConstPtr &graph) override;

  void visualizeImpl(tactic::QueryCache &,
                     const tactic::Graph::ConstPtr &) override;

  std::shared_ptr<Config> config_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr observation_map_pub_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr experience_map_pub_;
};

}  // namespace lidar
}  // namespace vtr