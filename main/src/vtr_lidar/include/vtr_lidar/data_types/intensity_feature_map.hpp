/**
 * \file intensity_feature_map.hpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * Sliding visual landmark map built from lidar intensity-image features.
 *
 * The analog of PointMap for ORB landmarks: features from successive
 * (motion-undistorted) scans are merged into a voxel-hashed landmark map
 * expressed in the odometry map frame. On a voxel hit, the descriptor
 * Hamming distance decides between "same landmark re-observed" (refresh
 * life time, average position, keep latest descriptor) and "different
 * landmark in the same voxel" (keep the stronger detector response).
 * Landmarks not re-observed for feature_life_time scans expire, mirroring
 * the point life time in OdometryMapMaintenanceModuleV2.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstring>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <unordered_map>
#include <vector>

#include "vtr_common/conversions/ros_lgmath.hpp"
#include "vtr_common/utils/macros.hpp"
#include "vtr_lidar/data_types/pointmap.hpp"  // pointmap::VoxKey + hash
#include "vtr_tactic/types.hpp"

#include "vtr_lidar_msgs/msg/intensity_feature_map.hpp"

namespace vtr {
namespace lidar {

class IntensityFeatureMap {
 public:
  PTR_TYPEDEFS(IntensityFeatureMap);

  using IntensityFeatureMapMsg = vtr_lidar_msgs::msg::IntensityFeatureMap;

  /** \brief Static function that constructs this class from a ROS2 message */
  static Ptr fromStorable(const IntensityFeatureMapMsg& storable);
  /** \brief Returns the ROS2 message to be stored */
  IntensityFeatureMapMsg toStorable() const;

  /// A single merged visual landmark in the map frame.
  struct Landmark {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();  ///< position in map frame
    cv::Mat descriptor;                           ///< 1 x 32 CV_8U ORB row
    float response = 0.0f;                        ///< detector response
    float life_time = 0.0f;                       ///< scans left to live
    int num_obs = 1;                              ///< observation count
  };

  IntensityFeatureMap(const float& dl, const float& descriptor_merge_dist)
      : dl_(dl), descriptor_merge_dist_(descriptor_merge_dist) {}

  float dl() const { return dl_; }
  size_t size() const { return landmarks_.size(); }
  const std::vector<Landmark>& landmarks() const { return landmarks_; }

  /** \brief Assembled descriptor matrix (one row per landmark). */
  cv::Mat descriptors() const {
    cv::Mat out;
    if (landmarks_.empty()) return out;
    out.create(static_cast<int>(landmarks_.size()),
               landmarks_.front().descriptor.cols,
               landmarks_.front().descriptor.type());
    for (size_t i = 0; i < landmarks_.size(); ++i)
      landmarks_[i].descriptor.copyTo(out.row(static_cast<int>(i)));
    return out;
  }

  /** \brief Landmark positions in the map frame (3 x N). */
  Eigen::Matrix<double, 3, Eigen::Dynamic> points() const {
    Eigen::Matrix<double, 3, Eigen::Dynamic> out(3, landmarks_.size());
    for (size_t i = 0; i < landmarks_.size(); ++i) out.col(i) = landmarks_[i].p;
    return out;
  }

  /**
   * \brief Merge a new scan's features into the map.
   * \param points       feature positions in the map frame (3 x N)
   * \param descriptors  ORB descriptors (N x 32, CV_8U)
   * \param responses    detector responses (size N)
   * \param life_time    life time assigned to inserted/re-observed landmarks
   */
  void update(const Eigen::Matrix<double, 3, Eigen::Dynamic>& points,
              const cv::Mat& descriptors, const std::vector<float>& responses,
              const float& life_time) {
    const int n = static_cast<int>(points.cols());
    if (n == 0 || descriptors.rows != n) return;
    landmarks_.reserve(landmarks_.size() + n);
    for (int i = 0; i < n; ++i) {
      const Eigen::Vector3d p = points.col(i);
      if (!p.allFinite()) continue;
      const float response =
          (i < static_cast<int>(responses.size())) ? responses[i] : 0.0f;
      const auto res = samples_.try_emplace(getKey(p), landmarks_.size());
      if (res.second) {
        landmarks_.emplace_back(
            Landmark{p, descriptors.row(i).clone(), response, life_time, 1});
        continue;
      }
      auto& lm = landmarks_[res.first->second];
      const double desc_dist =
          cv::norm(lm.descriptor, descriptors.row(i), cv::NORM_HAMMING);
      if (desc_dist <= descriptor_merge_dist_) {
        // same landmark re-observed: refresh and refine
        lm.num_obs++;
        lm.p += (p - lm.p) / static_cast<double>(lm.num_obs);
        lm.descriptor = descriptors.row(i).clone();  // track appearance
        lm.response = std::max(lm.response, response);
        lm.life_time = life_time;
      } else if (response > lm.response) {
        // different landmark in this voxel: keep the stronger one
        lm = Landmark{p, descriptors.row(i).clone(), response, life_time, 1};
      }
    }
  }

  /** \brief Decrement life times and remove expired landmarks. */
  void age(const float& decrement = 1.0f) {
    std::vector<Landmark> kept;
    kept.reserve(landmarks_.size());
    for (auto& lm : landmarks_) {
      lm.life_time -= decrement;
      if (lm.life_time > 0.0f) kept.emplace_back(std::move(lm));
    }
    landmarks_ = std::move(kept);
    // rebuild the voxel map; averaged positions may have migrated voxels,
    // so on a collision keep the first entry instead of throwing
    samples_.clear();
    samples_.reserve(landmarks_.size());
    for (size_t i = 0; i < landmarks_.size(); ++i)
      samples_.try_emplace(getKey(landmarks_[i].p), i);
  }

  tactic::VertexId& vertex_id() { return vertex_id_; }
  const tactic::VertexId& vertex_id() const { return vertex_id_; }
  tactic::EdgeTransform& T_vertex_this() { return T_vertex_this_; }
  const tactic::EdgeTransform& T_vertex_this() const { return T_vertex_this_; }

 private:
  using VoxKey = pointmap::VoxKey;
  VoxKey getKey(const Eigen::Vector3d& p) const {
    return VoxKey(static_cast<int>(std::floor(p.x() / dl_)),
                  static_cast<int>(std::floor(p.y() / dl_)),
                  static_cast<int>(std::floor(p.z() / dl_)));
  }

  /** \brief Voxel grid size */
  float dl_;
  /** \brief Max Hamming distance to merge as the same landmark */
  float descriptor_merge_dist_;
  /** \brief Merged landmarks */
  std::vector<Landmark> landmarks_;
  /** \brief Sparse hashmap from voxels to landmark indices */
  std::unordered_map<VoxKey, size_t> samples_;
  /** \brief The associated vertex id (set at vertex creation) */
  tactic::VertexId vertex_id_ = tactic::VertexId::Invalid();
  /** \brief Transform from the map frame to the vertex frame */
  tactic::EdgeTransform T_vertex_this_ = tactic::EdgeTransform(true);
};

inline auto IntensityFeatureMap::fromStorable(
    const IntensityFeatureMapMsg& storable) -> Ptr {
  auto data = std::make_shared<IntensityFeatureMap>(
      storable.dl, storable.descriptor_merge_dist);

  const int n = static_cast<int>(storable.num_landmarks);
  const int desc_cols =
      storable.descriptor_cols > 0 ? static_cast<int>(storable.descriptor_cols)
                                   : 32;
  data->landmarks_.reserve(n);
  for (int i = 0; i < n; ++i) {
    Landmark lm;
    lm.p << storable.points[i * 3 + 0], storable.points[i * 3 + 1],
        storable.points[i * 3 + 2];
    lm.descriptor = cv::Mat(1, desc_cols, CV_8U);
    std::memcpy(lm.descriptor.data, storable.descriptors.data() + i * desc_cols,
                desc_cols);
    lm.response = storable.responses[i];
    lm.life_time = storable.life_times[i];
    lm.num_obs = storable.num_obs[i];
    data->landmarks_.emplace_back(std::move(lm));
  }
  // rebuild the voxel map (tolerate collisions from averaged positions)
  data->samples_.reserve(data->landmarks_.size());
  for (size_t i = 0; i < data->landmarks_.size(); ++i)
    data->samples_.try_emplace(data->getKey(data->landmarks_[i].p), i);

  data->vertex_id_ = tactic::VertexId(storable.vertex_id);
  using namespace vtr::common;
  conversions::fromROSMsg(storable.t_vertex_this, data->T_vertex_this_);
  return data;
}

inline auto IntensityFeatureMap::toStorable() const -> IntensityFeatureMapMsg {
  IntensityFeatureMapMsg storable;
  storable.dl = dl_;
  storable.descriptor_merge_dist = descriptor_merge_dist_;

  const int n = static_cast<int>(landmarks_.size());
  const int desc_cols =
      landmarks_.empty() ? 32 : landmarks_.front().descriptor.cols;
  storable.num_landmarks = static_cast<uint32_t>(n);
  storable.descriptor_cols = static_cast<uint32_t>(desc_cols);
  storable.points.resize(n * 3);
  storable.descriptors.resize(n * desc_cols);
  storable.responses.resize(n);
  storable.life_times.resize(n);
  storable.num_obs.resize(n);
  for (int i = 0; i < n; ++i) {
    const auto& lm = landmarks_[i];
    storable.points[i * 3 + 0] = lm.p.x();
    storable.points[i * 3 + 1] = lm.p.y();
    storable.points[i * 3 + 2] = lm.p.z();
    std::memcpy(storable.descriptors.data() + i * desc_cols,
                lm.descriptor.data, desc_cols);
    storable.responses[i] = lm.response;
    storable.life_times[i] = lm.life_time;
    storable.num_obs[i] = lm.num_obs;
  }

  storable.vertex_id = vertex_id_;
  using namespace vtr::common;
  conversions::toROSMsg(T_vertex_this_, storable.t_vertex_this);
  return storable;
}

}  // namespace lidar
}  // namespace vtr
