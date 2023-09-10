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
 * \file costmap_inflation_module.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/planning/costmap_inflation_module.hpp"

#include "pcl/features/normal_3d.h"

#include "vtr_lidar/data_types/costmap.hpp"
#include "vtr_lidar/filters/voxel_downsample.hpp"

#include "vtr_lidar/utils/nanoflann_utils.hpp"

namespace vtr {
namespace lidar {

namespace {


template <typename PointT>
class DetectChangeOp {
 public:
  DetectChangeOp(const pcl::PointCloud<PointT> &points, const float &d0,
                 const float &d1)
      : d0_(d0), d1_(d1), adapter_(points) {
    /// create kd-tree of the point cloud for radius search
    kdtree_ = std::make_unique<KDTree<PointT>>(2, adapter_,
                                               KDTreeParams(/* max leaf */ 10));
    kdtree_->buildIndex();
    // search params setup
    search_params_.sorted = false;
  }

  void operator()(const Eigen::Vector2f &q, float &v) const {
    size_t ind;
    float dist;
    KDTreeResultSet result_set(1);
    result_set.init(&ind, &dist);
    kdtree_->findNeighbors(result_set, q.data(), search_params_);

    // update the value of v
    dist = std::sqrt(dist);  // convert to distance
    v = std::max(1 - (dist - d1_) / d0_, 0.0f);
    v = std::min(v, 0.9f);  // 1 is bad for visualization
  }

 private:
  const float d0_;
  const float d1_;

  KDTreeSearchParams search_params_;
  NanoFLANNAdapter<PointT> adapter_;
  std::unique_ptr<KDTree<PointT>> kdtree_;
};

}  // namespace

using namespace tactic;

auto CostmapInflationModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // cost map
  config->resolution = node->declare_parameter<float>(param_prefix + ".resolution", config->resolution);
  config->size_x = node->declare_parameter<float>(param_prefix + ".size_x", config->size_x);
  config->size_y = node->declare_parameter<float>(param_prefix + ".size_y", config->size_y);
  config->influence_distance = node->declare_parameter<float>(param_prefix + ".influence_distance", config->influence_distance);
  config->minimum_distance = node->declare_parameter<float>(param_prefix + ".minimum_distance", config->minimum_distance);
  // general
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void CostmapInflationModule::run_(QueryCache &qdata0, OutputCache &output0,
                                   const Graph::Ptr &graph,
                                   const TaskExecutor::Ptr & /* executor */) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);
  auto &output = dynamic_cast<LidarOutputCache &>(output0);

  /// visualization setup
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    costmap_pub_ = qdata.node->create_publisher<OccupancyGridMsg>("change_detection_costmap", 5);
    costpcd_pub_ = qdata.node->create_publisher<PointCloudMsg>("change_detection_costpcd", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  if (!qdata.changed_points.valid()) {
    CLOG(WARNING, "lidar.obstacle_inflation") << "No change detection run!";
    return;
  }

  // inputs
  const auto &stamp = *qdata.stamp;
  const auto &vid_loc = *qdata.vid_loc;
  const auto &sid_loc = *qdata.sid_loc;

  // clang-format off
  CLOG(INFO, "lidar.obstacle_inflation") << "Inflating obstacles at stamp: " << stamp;


  auto concat_pc = assemble_pointcloud(qdata0, output0, graph);

  // project to 2d and construct the grid map
  const auto costmap = std::make_shared<DenseCostMap>(
      config_->resolution, config_->size_x, config_->size_y);


  // update cost map based on change detection result
  DetectChangeOp<PointWithInfo> detect_change_op(
      concat_pc, config_->influence_distance, config_->minimum_distance);
  costmap->update(detect_change_op);
  // add transform to the localization vertex
  costmap->T_vertex_this() = tactic::EdgeTransform(true);
  costmap->vertex_id() = vid_loc;
  costmap->vertex_sid() = sid_loc;

  // Update the output cache
  output.costmap_sid = costmap->vertex_sid(); 
  output.obs_map = costmap->filter(0.01); //was config_->costmap_filter_value
  output.grid_resolution = config_->resolution;
  


  /// publish the transformed pointcloud
  if (config_->visualize) {

    // publish the occupancy grid
    auto costmap_msg = costmap->toCostMapMsg();
    costmap_msg.header.frame_id = "loc vertex frame";
    // costmap_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    costmap_pub_->publish(costmap_msg);

    // publish the point cloud
    auto pointcloud_msg = costmap->toPointCloudMsg();
    pointcloud_msg.header.frame_id = "loc vertex frame";
    // pointcloud_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    costpcd_pub_->publish(pointcloud_msg);
  }

  /// output
  auto change_detection_costmap_ref = output.change_detection_costmap.locked();
  auto &change_detection_costmap = change_detection_costmap_ref.get();
  change_detection_costmap = costmap;

  CLOG(INFO, "lidar.obstacle_inflation")
      << "Change detection for lidar scan at stamp: " << stamp << " - DONE";

}


pcl::PointCloud<PointWithInfo> CostmapInflationModule::assemble_pointcloud(tactic::QueryCache &qdata0, 
              tactic::OutputCache &, const tactic::Graph::Ptr &) {
  const auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);
  return *qdata.changed_points;
}

}  // namespace lidar
}  // namespace vtr