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
 * \file change_detection_module.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/planning/change_detection_module.hpp"

#include "vtr_lidar/data_types/costmap.hpp"
#include "vtr_lidar/utils/nanoflann_utils.hpp"

namespace vtr {
namespace lidar {

namespace {

template <class PointT>
void computeMeanAndStdDev(const pcl::PointCloud<PointT> &points, float &mean,
                          float &std_dev) {
  //
  const auto sum = std::accumulate(
      points.begin(), points.end(), 0.0,
      [](const float sum, const PointT &p) { return sum + p.z; });
  mean = sum / points.size();
  //
  const auto sq_sum =
      std::accumulate(points.begin(), points.end(), 0.0,
                      [mean](const float sq_sum, const PointT &p) {
                        return sq_sum + (p.z - mean) * (p.z - mean);
                      });
  const auto var = sq_sum / points.size();
  std_dev = var;
}

#if false
template <typename PointT>
class DetectChangeOp {
 public:
  DetectChangeOp(const pcl::PointCloud<PointT> &points,
                 const std::vector<float> &distances,
                 const std::vector<float> &roughnesses,
                 const float &search_radius)
      : points_(points),
        distances_(distances),
        roughnesses_(roughnesses),
        sq_search_radius_(search_radius * search_radius),
        adapter_(points) {
    /// create kd-tree of the point cloud for radius search
    kdtree_ = std::make_unique<KDTree<PointT>>(2, adapter_,
                                               KDTreeParams(/* max leaf */ 10));
    kdtree_->buildIndex();
    // search params setup
    search_params_.sorted = false;
  }

  void operator()(const Eigen::Vector2f &point, float &value) const {
    /// find the nearest neighbors
    std::vector<std::pair<size_t, float>> inds_dists;
    size_t num_neighbors = kdtree_->radiusSearch(
        point.data(), sq_search_radius_, inds_dists, search_params_);

    if (num_neighbors < 1) {
#if false
      CLOG(WARNING, "lidar.change_detection")
          << "looking at point: <" << x << "," << y << ">, roughness: 0"
          << " (no enough neighbors)";
#endif
      value = 10.0;  /// \todo arbitrary high value
      return;
    }
    std::vector<float> distances;
    std::vector<float> roughnesses;
    std::vector<float> neg_logprobs;
    distances.reserve(num_neighbors);
    roughnesses.reserve(num_neighbors);
    neg_logprobs.reserve(num_neighbors);
    for (size_t i = 0; i < num_neighbors; ++i) {
      distances.emplace_back(distances_[inds_dists[i].first]);
      roughnesses.emplace_back(std::sqrt(roughnesses_[inds_dists[i].first]));

      const auto &dist = distances_[inds_dists[i].first];
      const auto &rough = roughnesses_[inds_dists[i].first];
      // std::pow(dist, 2) / rough / 2 + std::log(std::sqrt(rough))
      neg_logprobs.emplace_back(std::pow(dist, 2) / (rough + 0.01) / 2.0);
    }
    // CLOG(DEBUG, "lidar.change_detection")
    //     << "\n Distance is " << distances << "\n roughness is " <<
    //     roughnesses;
    // use the negative log probability as the cost
    // value = *std::max_element(neg_logprobs.begin(), neg_logprobs.end());
    value = neg_logprobs[(size_t)std::floor((float)neg_logprobs.size() / 2)];
  }

 private:
  /** \brief reference to the point cloud */
  const pcl::PointCloud<PointT> &points_;
  const std::vector<float> &distances_;
  const std::vector<float> &roughnesses_;

  /** \brief squared search radius */
  const float sq_search_radius_;

  KDTreeSearchParams search_params_;
  NanoFLANNAdapter<PointT> adapter_;
  std::unique_ptr<KDTree<PointT>> kdtree_;
};
#endif
}  // namespace

using namespace tactic;

auto ChangeDetectionModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                            const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // change detection
  config->detection_range = node->declare_parameter<float>(param_prefix + "detection_range", config->detection_range);
  config->search_radius = node->declare_parameter<float>(param_prefix + ".search_radius", config->search_radius);
  // cost map
  config->resolution = node->declare_parameter<float>(param_prefix + ".resolution", config->resolution);
  config->size_x = node->declare_parameter<float>(param_prefix + ".size_x", config->size_x);
  config->size_y = node->declare_parameter<float>(param_prefix + ".size_y", config->size_y);
  // general
  config->run_online = node->declare_parameter<bool>(param_prefix + ".run_online", config->run_online);
  config->run_async = node->declare_parameter<bool>(param_prefix + ".run_async", config->run_async);
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  config->save_module_result = node->declare_parameter<bool>(param_prefix + ".save_module_result", config->save_module_result);
  // clang-format on
  return config;
}

void ChangeDetectionModule::run_(QueryCache &qdata0, OutputCache &output0,
                                 const Graph::Ptr &graph,
                                 const TaskExecutor::Ptr &executor) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);
  // auto &output = dynamic_cast<LidarOutputCache &>(output0);

  const auto &vid_loc = *qdata.vid_loc;

  if (config_->run_async)
    executor->dispatch(std::make_shared<Task>(
        shared_from_this(), qdata0.shared_from_this(), 0, Task::DepIdSet{},
        Task::DepId{}, "Change Detection", vid_loc));
  else
    runAsync_(qdata0, output0, graph, executor, Task::Priority(-1),
              Task::DepId());
}

void ChangeDetectionModule::runAsync_(QueryCache &qdata0, OutputCache &output0,
                                      const Graph::Ptr &graph,
                                      const TaskExecutor::Ptr &,
                                      const Task::Priority &,
                                      const Task::DepId &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);
  auto &output = dynamic_cast<LidarOutputCache &>(output0);

  if (config_->run_online &&
      output.chain->trunkSequenceId() != *qdata.sid_loc) {
    CLOG(INFO, "lidar.change_detection")
        << "Trunk id has changed, skip change detection for this scan";
    return;
  }

  /// save module result setup
  // centroid -> x, y, z
  // normal -> normal_x, normal_y, normal_z
  // query -> flex11, flex12, flex13
  // distance (point to plane) -> flex21
  // roughness -> flex22
  // number of observations -> flex23
  // fake point? -> flex24 (1 yes, 0 no)
  pcl::PointCloud<PointWithInfo> module_result;
  if (config_->save_module_result) module_result.reserve(5000);

  /// visualization setup
  if (config_->visualize) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!publisher_initialized_) {
      // clang-format off
      tf_bc_ = std::make_shared<tf2_ros::TransformBroadcaster>(*qdata.node);
      scan_pub_ = qdata.node->create_publisher<PointCloudMsg>("change_detection_scan", 5);
      map_pub_ = qdata.node->create_publisher<PointCloudMsg>("change_detection_map", 5);
      costmap_pub_ = qdata.node->create_publisher<OccupancyGridMsg>("change_detection_costmap", 5);
      pointcloud_pub_ = qdata.node->create_publisher<PointCloudMsg>("change_detection_pointcloud", 5);
      // clang-format on
      publisher_initialized_ = true;
    }
  }

  // inputs
  const auto &stamp = *qdata.stamp;
  const auto &T_s_r = *qdata.T_s_r;
  const auto &vid_loc = *qdata.vid_loc;
  const auto &sid_loc = *qdata.sid_loc;
  const auto &T_r_v_loc = *qdata.T_r_v_loc;
  const auto &points = *qdata.undistorted_point_cloud;
  const auto &submap_loc = *qdata.submap_loc;
  const auto &map_point_cloud = submap_loc.point_cloud();
  const auto &T_v_m_loc = *qdata.T_v_m_loc;

  CLOG(INFO, "lidar.change_detection")
      << "Change detection for lidar scan at stamp: " << stamp;

  // filter out points that are too far away
  std::vector<int> query_indices;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto &pt = points.at(i);
    if (pt.getVector3fMap().norm() < config_->detection_range)
      query_indices.emplace_back(i);
  }
  pcl::PointCloud<PointWithInfo> query_points(points, query_indices);

  // Eigen matrix of original data (only shallow copy of ref clouds)
  // clang-format off
  const auto query_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto query_norms_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());
  // clang-format on

  // retrieve the pre-processed scan and convert it to the localization frame
  pcl::PointCloud<PointWithInfo> aligned_points(query_points);
  // clang-format off
  auto aligned_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto aligned_norms_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());
  // clang-format on

  const auto T_m_s = (T_s_r * T_r_v_loc * T_v_m_loc).inverse().matrix();
  aligned_mat = T_m_s.cast<float>() * query_mat;
  aligned_norms_mat = T_m_s.cast<float>() * query_norms_mat;

  // create kd-tree of the map (2d only)
  NanoFLANNAdapter<PointWithInfo> adapter(map_point_cloud);
  KDTreeSearchParams search_params;
  KDTreeParams tree_params(10 /* max leaf */);
  auto kdtree =
      std::make_unique<KDTree<PointWithInfo>>(2, adapter, tree_params);
  kdtree->buildIndex();

  std::vector<float> nn_dists(aligned_points.size(), -1.0f);
  // compute point to plane distance
  const auto sq_search_radius = config_->search_radius * config_->search_radius;
  std::vector<float> roughnesses(aligned_points.size(), 0.0f);
  for (size_t i = 0; i < aligned_points.size(); i++) {
    // dump result
    if (config_->save_module_result) {
      auto &p = module_result.emplace_back();
      // set query point location info
      p.flex11 = aligned_points[i].x;
      p.flex12 = aligned_points[i].y;
      p.flex13 = aligned_points[i].z;
      //
      p.flex14 = -1.0;  // invalid normal agreement
      //
      p.flex21 = -1.0;                      // invalid distance
      p.flex22 = -1.0;                      // invalid roughness
      p.flex23 = -1.0;                      // invalid number of observations
      p.flex24 = aligned_points[i].flex24;  // fake point copied directly
    }

    // radius search of the closest point
    std::vector<std::pair<size_t, float>> inds_dists;
    kdtree->radiusSearch(aligned_points[i].data, sq_search_radius, inds_dists,
                         search_params);
    std::vector<int> indices;
    indices.reserve(inds_dists.size());
    for (const auto &ind_dist : inds_dists) indices.push_back(ind_dist.first);

    // filter based on neighbors in map /// \todo parameters
    if (indices.size() < 10) continue;

    // compute the planar distance
    float mean;
    computeMeanAndStdDev(
        pcl::PointCloud<PointWithInfo>(map_point_cloud, indices), mean,
        roughnesses[i]);

    const auto diff = aligned_points[i].z - mean;
    nn_dists[i] = std::abs(diff);

    // save module result
    if (config_->save_module_result) {
      auto &p = module_result.back();
      // set query point location info
      // clang-format off
      p.getVector3fMap() = Eigen::Vector3f{aligned_points[i].x, aligned_points[i].y, mean};
      p.getNormalVector3fMap() = Eigen::Vector3f{0, 0, 1};
      p.flex21 = nn_dists[i];
      p.flex22 = roughnesses[i];
      p.flex23 = static_cast<float>(indices.size());
      // clang-format on
    }
  }

  for (size_t i = 0; i < aligned_points.size(); i++) {
    aligned_points[i].flex23 = 0.0f;
    if (nn_dists[i] < 0.0 || (std::pow(nn_dists[i], 2) > roughnesses[i]))
      aligned_points[i].flex23 = 1.0f;
  }

  // retrieve the pre-processed scan and convert it to the robot frame
  pcl::PointCloud<PointWithInfo> aligned_points2(aligned_points);
  // clang-format off
  auto aligned_mat2 = aligned_points2.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto aligned_norms_mat2 = aligned_points2.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());
  // clang-format on

  const auto T_r_m = (T_m_s * T_s_r).inverse().matrix();
  aligned_mat2 = T_r_m.cast<float>() * aligned_mat;
  aligned_norms_mat2 = T_r_m.cast<float>() * aligned_norms_mat;

#if false
  // project to 2d and construct the grid map
  const auto costmap = std::make_shared<DenseCostMap>(
      config_->resolution, config_->size_x, config_->size_y);
  // update cost map based on change detection result
  DetectChangeOp<PointWithInfo> detect_change_op(
      aligned_points2, nn_dists, nn_roughnesses, config_->search_radius);
  costmap->update(detect_change_op);
  // add transform to the localization vertex
  costmap->T_vertex_this() = T_r_v_loc.inverse();
  costmap->vertex_id() = vid_loc;
  costmap->vertex_sid() = sid_loc;
#endif

  ///
  if (config_->save_module_result) {
    // save result
    CLOG(INFO, "lidar.change_detection") << "Saving change detection result";
    const auto ros_msg = std::make_shared<PointCloudMsg>();
    pcl::toROSMsg(module_result, *ros_msg);
    using PoincCloudMsgLM = storage::LockableMessage<PointCloudMsg>;
    auto msg = std::make_shared<PoincCloudMsgLM>(ros_msg, *qdata.stamp);
    graph->write<PointCloudMsg>("change_detection_result",
                                "sensor_msgs/msg/PointCloud2", msg);
  }

  /// publish the transformed pointcloud
  if (config_->visualize) {
    std::unique_lock<std::mutex> lock(mutex_);
    //
    const auto T_w_v_loc = config_->run_online
                               ? output.chain->pose(*qdata.sid_loc)  // online
                               : T_v_m_loc.inverse();                // offline

    if (!config_->run_online) {
      // publish the aligned points
      PointCloudMsg scan_msg;
      pcl::toROSMsg(aligned_points, scan_msg);
      scan_msg.header.frame_id = "world";
      // scan_msg.header.stamp = rclcpp::Time(*qdata.stamp);
      scan_pub_->publish(scan_msg);

      // publish the submap for localization
      PointCloudMsg submap_msg;
      pcl::toROSMsg(map_point_cloud, submap_msg);
      submap_msg.header.frame_id = "world (offset)";
      // submap_msg.header.stamp = rclcpp::Time(*qdata.stamp);
      map_pub_->publish(submap_msg);
    }
#if false
    // publish the occupancy grid origin
    Eigen::Affine3d T((T_w_v_loc * T_r_v_loc.inverse()).matrix());
    auto tf_msg = tf2::eigenToTransform(T);
    tf_msg.header.frame_id = "world (offset)";
    // tf_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    tf_msg.child_frame_id = "change detection";
    tf_bc_->sendTransform(tf_msg);

    // publish the occupancy grid
    auto costmap_msg = costmap->toCostMapMsg();
    costmap_msg.header.frame_id = "change detection";
    // costmap_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    costmap_pub_->publish(costmap_msg);

    // publish the point cloud
    auto pointcloud_msg = costmap->toPointCloudMsg();
    pointcloud_msg.header.frame_id = "change detection";
    // pointcloud_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    pointcloud_pub_->publish(pointcloud_msg);
#endif
  }

  /// output
#if false
  auto change_detection_costmap_ref = output.change_detection_costmap.locked();
  auto &change_detection_costmap = change_detection_costmap_ref.get();
  change_detection_costmap = costmap;
#endif

  CLOG(INFO, "lidar.change_detection")
      << "Change detection for lidar scan at stamp: " << stamp << " - DONE";
}

}  // namespace lidar
}  // namespace vtr