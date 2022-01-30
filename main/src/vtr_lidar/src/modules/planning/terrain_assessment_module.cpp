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
 * \file terrain_assessment_module.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/planning/terrain_assessment_module.hpp"

#include "pcl/features/normal_3d.h"

namespace vtr {
namespace lidar {

namespace {

class ComputeCorridorOp {
 public:
  ComputeCorridorOp(const unsigned &curr_sid,
                    const tactic::LocalizationChain &chain,
                    const float &lookahead_distance, const float &width)
      : lookahead_distance_(lookahead_distance), width_(width) {
    auto lock = chain.guard();
    // compute keyframe lookahead
    const auto distance = chain.dist(curr_sid);
    const auto T_w_curr = chain.pose(curr_sid);
    for (auto query_sid = curr_sid;
         query_sid < chain.size() &&
         (chain.dist(query_sid) - distance) < lookahead_distance_;
         ++query_sid) {
      const auto T_curr_query = T_w_curr.inverse() * chain.pose(query_sid);
      T_curr_query_vec.emplace_back(T_curr_query.matrix());
      T_curr_query_xy_vec.emplace_back(
          T_curr_query.matrix().block<2, 1>(0, 3).cast<float>());
#if false
      CLOG(DEBUG, "lidar.terrain_assessment")
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
    CLOG(DEBUG, "lidar.terrain_assessment")
        << "distances of: <" << q(0) << "," << q(1) << ">: " << distances;
#endif
    // update the value of v
    v = min_dist > width_ ? v : 1;
  }

  std::vector<Eigen::Matrix4d> T_curr_query_vec;
  std::vector<Eigen::Vector2f> T_curr_query_xy_vec;

 private:
  const float lookahead_distance_;
  const float width_;
};

template <typename PointT>
class AssessTerrainOp {
 public:
  AssessTerrainOp(const pcl::PointCloud<PointT> &points,
                  const float &search_radius)
      : points_(points),
        sq_search_radius_(search_radius * search_radius),
        adapter_(points) {
    /// create kd-tree of the point cloud for radius search
    kdtree_ = std::make_unique<KDTree<PointT>>(2, adapter_,
                                               KDTreeParams(10 /* max leaf */));
    kdtree_->buildIndex();
    // search params setup
    search_params_.sorted = false;
  }

  void operator()(const Eigen::Vector2f &point, float &value) const {
    /// find the nearest neighbors
    std::vector<std::pair<size_t, float>> inds_dists;
    size_t num_neighbors = kdtree_->radiusSearch(
        point.data(), sq_search_radius_, inds_dists, search_params_);

    if (num_neighbors < 5) {
#if false
      CLOG(WARNING, "lidar.terrain_assessment")
          << "looking at point: <" << x << "," << y << ">, roughness: 0"
          << " (no enough neighbors)";
#endif
      // \todo 0.0 should not mean no enough neighbors
      value = 0.0;
      return;
    }

    std::vector<int> indices(num_neighbors);
    for (size_t i = 0; i < num_neighbors; i++) indices[i] = inds_dists[i].first;

    /// apply pca to compute the roughness
    // get points for computation
    const pcl::PointCloud<PointT> query_points(points_, indices);

    // Placeholder for the 3x3 covariance matrix at each surface patch
    Eigen::Matrix3f covariance_matrix;
    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4f xyz_centroid;

    // Estimate the XYZ centroid
    pcl::compute3DCentroid(query_points, xyz_centroid);
    // Compute the 3x3 covariance matrix
    pcl::computeCovarianceMatrix(query_points, xyz_centroid, covariance_matrix);

    // Compute pca
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
    es.compute(covariance_matrix);

    // compute the roughness (smallest eigenvalue)
    float roughness = std::abs(es.eigenvalues()(0));
#if false
    // compute the slope of the surface (1 is vertical, 0 is horizontal)
    float slope = std::abs(es.eigenvectors()(0, 2));
    // compute the step height (max difference between points)
    std::vector<float> z_values(num_neighbors);
    for (size_t i = 0; i < num_neighbors; i++) z_values[i] = query_points[i].z;
    std::sort(z_values.begin(), z_values.end());
    // \todo consider using the 90 percentile
    float step_height = z_values[num_neighbors - 1] - z_values[0];
#endif
#if false
    CLOG(DEBUG, "lidar.terrain_assessment")
        << "looking at point: <" << x << "," << y
        << ">, roughness: " << roughness;
#endif
    value = roughness;  /// \todo use slope and step height as well
  }

 private:
  /** \brief reference to the point cloud */
  const pcl::PointCloud<PointT> &points_;

  /** \brief squared search radius */
  const float sq_search_radius_;

  KDTreeSearchParams search_params_;
  NanoFLANNAdapter<PointT> adapter_;
  std::unique_ptr<KDTree<PointT>> kdtree_;
};

}  // namespace

using namespace tactic;

auto TerrainAssessmentModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // corridor computation
  config->corridor_lookahead_distance = node->declare_parameter<float>(param_prefix + ".corridor_lookahead_distance", config->corridor_lookahead_distance);
  config->corridor_width = node->declare_parameter<float>(param_prefix + ".corridor_width", config->corridor_width);
  // terrain assessment
  config->search_radius = node->declare_parameter<float>(param_prefix + ".search_radius", config->search_radius);
  // cost map
  config->resolution = node->declare_parameter<float>(param_prefix + ".resolution", config->resolution);
  config->size_x = node->declare_parameter<float>(param_prefix + ".size_x", config->size_x);
  config->size_y = node->declare_parameter<float>(param_prefix + ".size_y", config->size_y);
  // general
  config->run_online = node->declare_parameter<bool>(param_prefix + ".run_online", config->run_online);
  config->run_async = node->declare_parameter<bool>(param_prefix + ".run_async", config->run_async);
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

TerrainAssessmentModule::TerrainAssessmentModule(
    const Config::ConstPtr &config,
    const std::shared_ptr<tactic::ModuleFactory> &module_factory,
    const std::string &name)
    : tactic::BaseModule{module_factory, name}, config_(config) {}

void TerrainAssessmentModule::run_(QueryCache &qdata0, OutputCache &output0,
                                   const Graph::Ptr &graph,
                                   const TaskExecutor::Ptr &executor) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);
  // auto &output = dynamic_cast<LidarOutputCache &>(output0);

  const auto &map_id = *qdata.map_id;
  if (qdata.curr_map_loc_changed && (!(*qdata.curr_map_loc_changed))) {
    CLOG(DEBUG, "lidar.terrain_assessment")
        << "Terrain Assessment for vertex " << map_id << " is already done.";
    return;
  }

  if (config_->run_async)
    executor->dispatch(std::make_shared<Task>(
        shared_from_this(), qdata.shared_from_this(), 0, Task::DepIdSet{},
        Task::DepId{}, "Terrain Assessment", map_id));
  else
    runAsync_(qdata0, output0, graph, executor, Task::Priority(-1),
              Task::DepId());
}

void TerrainAssessmentModule::runAsync_(
    QueryCache &qdata0, OutputCache &output0, const Graph::Ptr &,
    const TaskExecutor::Ptr &, const Task::Priority &, const Task::DepId &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);
  auto &output = dynamic_cast<LidarOutputCache &>(output0);

  if (config_->visualize) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!publisher_initialized_) {
      // clang-format off
      tf_bc_ = std::make_shared<tf2_ros::TransformBroadcaster>(*qdata.node);
      map_pub_ = qdata.node->create_publisher<PointCloudMsg>("terrain_assessment", 5);
      path_pub_ = qdata.node->create_publisher<PathMsg>("terrain_assessment_path", 5);
      costmap_pub_ = qdata.node->create_publisher<OccupancyGridMsg>("terrain_assessment_costmap", 5);
      pointcloud_pub_ = qdata.node->create_publisher<PointCloudMsg>("terrain_assessment_pointcloud", 5);
      // clang-format on
      publisher_initialized_ = true;
    }
  }

  if (config_->run_online &&
      output.chain->trunkSequenceId() != *qdata.map_sid) {
    CLOG(INFO, "lidar.terrain_assessment")
        << "Trunk id has changed, skip change detection for this scan";
    return;
  }

  // input
  const auto &chain = *output.chain;
  const auto &loc_vid = *qdata.map_id;
  const auto &loc_sid = *qdata.map_sid;
  const auto &point_map = *qdata.curr_map_loc;
  const auto &T_lv_pm = point_map.T_vertex_map().matrix();
  auto point_cloud = point_map.point_map();  // copy for changing

  CLOG(INFO, "lidar.terrain_assessment")
      << "Terrain Assessment for vertex: " << loc_vid;

  // transform into vertex frame
  // clang-format off
  auto points_mat = point_cloud.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto normal_mat = point_cloud.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::normal_offset());
  // clang-format on
  Eigen::Matrix3f C_lv_pm = (T_lv_pm.block<3, 3>(0, 0)).cast<float>();
  Eigen::Vector3f r_lv_pm = (T_lv_pm.block<3, 1>(0, 3)).cast<float>();
  points_mat = ((C_lv_pm * points_mat).colwise() + r_lv_pm).eval();
  normal_mat = (C_lv_pm * normal_mat).eval();

  // construct the cost map
  const auto costmap = std::make_shared<DenseCostMap>(
      config_->resolution, config_->size_x, config_->size_y);
  // // update cost map based on terrain assessment result
  // AssessTerrainOp<PointWithInfo> assess_terrain_op(point_cloud,
  //                                                  config_->search_radius);
  // costmap->update(assess_terrain_op);
  // mask out the robot footprint during teach pass
  ComputeCorridorOp compute_corridor_op(loc_sid, chain,
                                        config_->corridor_lookahead_distance,
                                        config_->corridor_width);
  costmap->update(compute_corridor_op);
  // add transform to the localization vertex
  costmap->T_vertex_this() = tactic::EdgeTransform(true);
  costmap->vertex_id() = loc_vid;
  costmap->vertex_sid() = loc_sid;

  /// publish the transformed pointcloud
  if (config_->visualize) {
    std::unique_lock<std::mutex> lock(mutex_);
    //
    const auto T_w_lv = config_->run_online
                            ? output.chain->pose(*qdata.map_sid)  // online
                            : EdgeTransform(true);                // offline

    // publish the occupancy grid origin
    Eigen::Affine3d T(T_w_lv.matrix());
    auto msg = tf2::eigenToTransform(T);
    msg.header.frame_id = "world (offset)";
    // msg.header.stamp = rclcpp::Time(*qdata.stamp);
    msg.child_frame_id = "terrain assessment";
    tf_bc_->sendTransform(msg);

    if (!config_->run_online) {
      // publish the transformed map (now in vertex frame)
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(point_cloud, pc2_msg);
      pc2_msg.header.frame_id = "world (offset)";
      // pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
      map_pub_->publish(pc2_msg);

      // publish the teach path
      PathMsg path_msg;
      path_msg.header.frame_id = "terrain assessment";
      for (unsigned i = 0; i < compute_corridor_op.T_curr_query_vec.size();
           ++i) {
        auto &pose = path_msg.poses.emplace_back();
        pose.pose = tf2::toMsg(
            Eigen::Affine3d(compute_corridor_op.T_curr_query_vec[i]));
      }
      path_pub_->publish(path_msg);
    }

    // publish the occupancy grid
    auto costmap_msg = costmap->toCostMapMsg();
    costmap_msg.header.frame_id = "terrain assessment";
    // costmap_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    costmap_pub_->publish(costmap_msg);

    // publish the point cloud
    auto pointcloud_msg = costmap->toPointCloudMsg();
    pointcloud_msg.header.frame_id = "terrain assessment";
    // pointcloud_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    pointcloud_pub_->publish(pointcloud_msg);
  }

  /// output
  auto terrain_assessment_costmap_ref =
      output.terrain_assessment_costmap.locked();
  auto &terrain_assessment_costmap = terrain_assessment_costmap_ref.get();
  terrain_assessment_costmap = costmap;

  CLOG(INFO, "lidar.terrain_assessment")
      << "Terrain Assessment for vertex: " << loc_vid << " - DONE!";
}

}  // namespace lidar
}  // namespace vtr