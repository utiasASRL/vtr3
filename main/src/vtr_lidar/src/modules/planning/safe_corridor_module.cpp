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
 * \file safe_corridor_module.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/planning/safe_corridor_module.hpp"

#include "vtr_lidar/utils/nanoflann_utils.hpp"

namespace vtr {
namespace lidar {

namespace {

class ComputeCorridorOp {
 public:
  ComputeCorridorOp(const unsigned &curr_sid,
                    const tactic::LocalizationChain &chain,
                    const float &lookahead_distance, const float &width,
                    const float &d0)
      : lookahead_distance_(lookahead_distance), width_(width), d0_(d0) {
    auto lock = chain.guard();
    // compute vertex lookahead
    const auto distance = chain.dist(curr_sid);
    const auto T_w_curr = chain.pose(curr_sid);
    // forwards
    for (auto query_sid = curr_sid;
         query_sid < chain.size() &&
         (chain.dist(query_sid) - distance) < lookahead_distance_;
         ++query_sid) {
      const auto T_curr_query = T_w_curr.inverse() * chain.pose(query_sid);
      T_curr_query_vec.emplace_back(T_curr_query.matrix());
      T_curr_query_xy_vec.emplace_back(
          T_curr_query.matrix().block<2, 1>(0, 3).cast<float>());
#if false
      CLOG(DEBUG, "lidar.safe_corridor")
          << "query sequence id: " << query_sid
          << ", T_curr_query: " << T_curr_query.vec().transpose();
#endif
    }
  }

  void operator()(const Eigen::Vector2f &q, float &v) const {
    // use the following convention (all points are (x, y)):
    //   q  - query point (center of the cell)
    //   p  - projected point on to the line segment
    //   xs - start point of the line segment
    //   xe - end point of the line segment
    std::vector<float> distances;
    distances.reserve(T_curr_query_xy_vec.size() + 1);

    // distance to the first vertex along the map
    distances.emplace_back((q - T_curr_query_xy_vec.front()).norm());
    // distance to intermediate line segments
    if (T_curr_query_xy_vec.size() > 1) {
      for (size_t i = 0; i < T_curr_query_xy_vec.size() - 1; ++i) {
        const auto &xs = T_curr_query_xy_vec[i];
        const auto &xe = T_curr_query_xy_vec[i + 1];
        float alpha = (q - xs).dot(xe - xs) / (xe - xs).squaredNorm();
        alpha = std::clamp(alpha, 0.0f, 1.0f);
        const auto p = xs + alpha * (xe - xs);
        distances.emplace_back((q - p).norm());
      }
    }
    // distance to the last vertex along the path
    distances.emplace_back((q - T_curr_query_xy_vec.back()).norm());

    // find the minimum distance
    const auto min_dist = *std::min_element(distances.begin(), distances.end());
#if false
    CLOG(DEBUG, "lidar.safe_corridor")
        << "distances of: <" << q(0) << "," << q(1) << ">: " << distances;
#endif
    // update the value of v
    v = std::max(1 - (width_ - min_dist) / d0_, 0.0f);
  }

  std::vector<Eigen::Matrix4d> T_curr_query_vec;
  std::vector<Eigen::Vector2f> T_curr_query_xy_vec;

 private:
  const float lookahead_distance_;
  const float width_;
  const float d0_;
};

}  // namespace

using namespace tactic;

auto SafeCorridorModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                         const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // corridor computation
  config->corridor_lookahead_distance = node->declare_parameter<float>(param_prefix + ".corridor_lookahead_distance", config->corridor_lookahead_distance);
  config->corridor_width = node->declare_parameter<float>(param_prefix + ".corridor_width", config->corridor_width);
  config->influence_distance = node->declare_parameter<float>(param_prefix + ".influence_distance", config->influence_distance);
  // cost map
  config->resolution = node->declare_parameter<float>(param_prefix + ".resolution", config->resolution);
  config->size_x = node->declare_parameter<float>(param_prefix + ".size_x", config->size_x);
  config->size_y = node->declare_parameter<float>(param_prefix + ".size_y", config->size_y);
  // general
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

SafeCorridorModule::SafeCorridorModule(
    const Config::ConstPtr &config,
    const std::shared_ptr<tactic::ModuleFactory> &module_factory,
    const std::string &name)
    : tactic::BaseModule{module_factory, name}, config_(config) {}

void SafeCorridorModule::run_(QueryCache &qdata0, OutputCache &output0,
                              const Graph::Ptr &graph,
                              const TaskExecutor::Ptr &executor) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);
  auto &output = dynamic_cast<LidarOutputCache &>(output0);

  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    costmap_pub_ = qdata.node->create_publisher<OccupancyGridMsg>("safe_corridor_costmap", 5);
    pointcloud_pub_ = qdata.node->create_publisher<PointCloudMsg>("safe_corridor_pointcloud", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  // input
  const auto &chain = *output.chain;
  const auto &loc_vid = *qdata.vid_loc;
  const auto &loc_sid = *qdata.sid_loc;

  // construct the cost map
  const auto costmap = std::make_shared<DenseCostMap>(
      config_->resolution, config_->size_x, config_->size_y);

  // mask out the robot footprint during teach pass
  ComputeCorridorOp compute_corridor_op(
      loc_sid, chain, config_->corridor_lookahead_distance,
      config_->corridor_width, config_->influence_distance);
  costmap->update(compute_corridor_op);
  // add transform to the localization vertex
  costmap->T_vertex_this() = tactic::EdgeTransform(true);
  costmap->vertex_id() = loc_vid;
  costmap->vertex_sid() = loc_sid;

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
    pointcloud_pub_->publish(pointcloud_msg);
  }

  /// output
  auto safe_corridor_costmap_ref = output.safe_corridor_costmap.locked();
  auto &safe_corridor_costmap = safe_corridor_costmap_ref.get();
  safe_corridor_costmap = costmap;
}

}  // namespace lidar
}  // namespace vtr