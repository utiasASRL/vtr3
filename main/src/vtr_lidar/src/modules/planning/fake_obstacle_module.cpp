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
 * \file fake_obstacle_module.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/planning/fake_obstacle_module.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

auto FakeObstacleModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                         const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // object path
  config->path = node->declare_parameter<std::string>(param_prefix + ".path", std::string{});
  config->objs = node->declare_parameter<std::vector<std::string>>(param_prefix + ".objs", std::vector<std::string>{});
  // object locations
  config->fixed_types = node->declare_parameter<std::vector<long int>>(param_prefix + ".types", std::vector<long int>{});
  config->fixed_xs = node->declare_parameter<std::vector<double>>(param_prefix + ".xs", std::vector<double>{});
  config->fixed_ys = node->declare_parameter<std::vector<double>>(param_prefix + ".ys", std::vector<double>{});
  config->fixed_zs = node->declare_parameter<std::vector<double>>(param_prefix + ".zs", std::vector<double>{});
  config->fixed_rolls = node->declare_parameter<std::vector<double>>(param_prefix + ".rolls", std::vector<double>{});
  config->fixed_pitchs = node->declare_parameter<std::vector<double>>(param_prefix + ".pitchs", std::vector<double>{});
  config->fixed_yaws = node->declare_parameter<std::vector<double>>(param_prefix + ".yaws", std::vector<double>{});
  // general
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

FakeObstacleModule::FakeObstacleModule(
    const Config::ConstPtr &config,
    const std::shared_ptr<tactic::ModuleFactory> &module_factory,
    const std::string &name)
    : tactic::BaseModule{module_factory, name}, config_(config) {
  // hard-coded honeycomb
  mesh2pcd::Mesh2PcdConverter::Config m2p_config;
  m2p_config.theta_min = 1.204;
  m2p_config.theta_res = 0.013;
  m2p_config.theta_max = 2.862;
  m2p_config.phi_min = -1.833;
  m2p_config.phi_max = 1.833;
  m2p_config.phi_res = 0.021;
  m2p_config.range_min = 2.0;
  m2p_config.range_max = 40.0;

  for (const auto &obj : config_->objs) {
    const std::string filename = config_->path + "/" + obj + ".obj";
    CLOG(WARNING, "test") << "Loading obj file: " << filename;
    converters_.emplace_back(filename, m2p_config);
  }

  const auto num_fixed_objs = config_->fixed_types.size();
  for (size_t i = 0; i < num_fixed_objs; ++i) {
    Eigen::Matrix<double, 6, 1> T_scan_obj_vec;
    // clang-format off
    T_scan_obj_vec << config_->fixed_xs.at(i),
                     config_->fixed_ys.at(i),
                     config_->fixed_zs.at(i),
                     config_->fixed_rolls.at(i),
                     config_->fixed_pitchs.at(i),
                     config_->fixed_yaws.at(i);
    // clang-format on
    const auto T_scan_obj = lgmath::se3::vec2tran(T_scan_obj_vec);
    obj_T_scan_objs_.emplace_back(config_->fixed_types.at(i),
                                  T_scan_obj.cast<float>());
  }
}

void FakeObstacleModule::run_(QueryCache &qdata0, OutputCache &output0,
                              const Graph::Ptr &graph,
                              const TaskExecutor::Ptr &executor) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    pointcloud_pub_ = qdata.node->create_publisher<PointCloudMsg>("fake_obstacle_scan", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  auto point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(
      *qdata.undistorted_point_cloud);

  for (size_t i = 0; i < obj_T_scan_objs_.size(); ++i) {
    const auto &obj = obj_T_scan_objs_.at(i).first;
    const auto &T_scan_obj = obj_T_scan_objs_.at(i).second;
    converters_.at(obj).addToPcd(*point_cloud, T_scan_obj, i == 0);
  }

  qdata.undistorted_point_cloud = point_cloud;

  /// publish the updated pointcloud
  if (config_->visualize) {
    // remove static points
    std::vector<int> indices;
    indices.reserve(point_cloud->size());
    for (size_t i = 0; i < point_cloud->size(); ++i) {
      /// \todo change flex24 to flex23
      if ((*point_cloud)[i].flex24 > 0.5f) indices.emplace_back(i);
    }
    pcl::PointCloud<PointWithInfo> filtered_points(*point_cloud, indices);

    // publish the aligned points
    PointCloudMsg scan_msg;
    pcl::toROSMsg(filtered_points, scan_msg);
    scan_msg.header.frame_id = "lidar";
    // scan_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    pointcloud_pub_->publish(scan_msg);
  }
}

}  // namespace lidar
}  // namespace vtr