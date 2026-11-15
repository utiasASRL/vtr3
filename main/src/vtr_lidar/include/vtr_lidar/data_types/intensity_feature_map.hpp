/**
 * \file intensity_feature_map.hpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * Sliding visual landmark map built from lidar intensity-image features.
 *
 * The analog of PointMap for ORB landmarks, expressed in the odometry map
 * frame. Landmarks are merged by association instead of a voxel grid: the
 * position error of a feature grows with its range (one pixel is 6 mrad:
 * 2 cm at 3 m, 25 cm at 40 m), so a fixed voxel is too small far away
 * (re-observations are inserted again and again) and too large close by
 * (different keypoints share a voxel).
 *
 *   - a feature that the odometry matched to a landmark (mutual nearest
 *     neighbour + MD-RANSAC inlier) updates that landmark;
 *   - any other feature is compared with the landmarks around it: if one is
 *     within the Mahalanobis gate
 *         (p - p_lm)^T (Sigma + Sigma_lm)^-1 (p - p_lm) < merge_gate
 *     and its descriptor is close enough (descriptor_merge_dist), it is the
 *     same landmark; otherwise the feature becomes a new landmark.
 *
 * Every landmark keeps a 3x3 covariance (map frame), needed by the feature
 * residual. When a landmark is observed again it keeps the position and
 * covariance of its most accurate observation (smallest covariance trace)
 * and the latest descriptor. Landmarks not re-observed for feature_life_time
 * scans expire, mirroring the point life time in
 * OdometryMapMaintenanceModuleV2.
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
    /// covariance of the position in the map frame [m^2]
    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
    cv::Mat descriptor;                           ///< 1 x 32 CV_8U ORB row
    float response = 0.0f;                        ///< detector response
    float life_time = 0.0f;                       ///< scans left to live
    int num_obs = 1;                              ///< observation count
  };

  /// Counts of the last update() (for logging).
  struct UpdateStats {
    int matched = 0;  ///< features that came with a landmark from odometry
    int merged = 0;   ///< unmatched features merged into a nearby landmark
    int added = 0;    ///< new landmarks
  };

  /**
   * \param merge_gate  squared Mahalanobis gate of a duplicate (11.34: 99 %
   *                    of chi^2 with 3 dof)
   * \param merge_radius  landmarks further away are not compared [m]
   * \param descriptor_merge_dist  max Hamming distance of a duplicate
   */
  IntensityFeatureMap(const float& merge_gate, const float& merge_radius,
                      const float& descriptor_merge_dist)
      : merge_gate_(merge_gate),
        merge_radius_(merge_radius),
        descriptor_merge_dist_(descriptor_merge_dist) {}

  size_t size() const { return landmarks_.size(); }
  const std::vector<Landmark>& landmarks() const { return landmarks_; }
  const UpdateStats& last_update() const { return last_update_; }

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
   * \param covs         their covariances in the map frame (size N)
   * \param descriptors  ORB descriptors (N x 32, CV_8U)
   * \param responses    detector responses (size N)
   * \param matched_landmark  index of the landmark the odometry matched each
   *                     feature to (-1 = none); empty = no feature is matched
   * \param life_time    life time assigned to inserted/re-observed landmarks
   */
  void update(const Eigen::Matrix<double, 3, Eigen::Dynamic>& points,
              const std::vector<Eigen::Matrix3d>& covs,
              const cv::Mat& descriptors, const std::vector<float>& responses,
              const std::vector<int>& matched_landmark,
              const float& life_time) {
    last_update_ = UpdateStats();
    const int n = static_cast<int>(points.cols());
    if (n == 0 || descriptors.rows != n ||
        static_cast<int>(covs.size()) != n)
      return;
    const int num_old = static_cast<int>(landmarks_.size());

    // (1) the landmark every feature belongs to (-1 = a new landmark)
    std::vector<int> target(n, -1);
    std::vector<bool> usable(n, false);
    const bool has_matches = static_cast<int>(matched_landmark.size()) == n;
    // coarse grid over the existing landmarks to find the ones near a feature
    std::unordered_map<VoxKey, std::vector<int>> grid;
    const double cell = std::max(static_cast<double>(merge_radius_), 1e-3);
    const auto cellOf = [&](const Eigen::Vector3d& p) {
      return VoxKey(static_cast<int>(std::floor(p.x() / cell)),
                    static_cast<int>(std::floor(p.y() / cell)),
                    static_cast<int>(std::floor(p.z() / cell)));
    };
    for (int j = 0; j < num_old; ++j)
      grid[cellOf(landmarks_[j].p)].push_back(j);
    const double radius2 =
        static_cast<double>(merge_radius_) * static_cast<double>(merge_radius_);

    for (int i = 0; i < n; ++i) {
      const Eigen::Vector3d p = points.col(i);
      if (!p.allFinite() || !covs[i].allFinite()) continue;
      usable[i] = true;
      if (has_matches && matched_landmark[i] >= 0 &&
          matched_landmark[i] < num_old) {
        target[i] = matched_landmark[i];
        last_update_.matched++;
        continue;
      }
      // duplicate of a nearby landmark: Mahalanobis gate + descriptor; of
      // several candidates the one with the closest descriptor
      const VoxKey key = cellOf(p);
      double best_dist = static_cast<double>(descriptor_merge_dist_);
      int best = -1;
      for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
          for (int dz = -1; dz <= 1; ++dz) {
            const auto it = grid.find(key + VoxKey(dx, dy, dz));
            if (it == grid.end()) continue;
            for (const int j : it->second) {
              const auto& lm = landmarks_[j];
              const Eigen::Vector3d d = p - lm.p;
              if (d.squaredNorm() > radius2) continue;
              const double desc_dist =
                  cv::norm(lm.descriptor, descriptors.row(i), cv::NORM_HAMMING);
              if (desc_dist > best_dist) continue;
              const Eigen::Matrix3d S = covs[i] + lm.cov;
              const double maha = d.dot(S.ldlt().solve(d));
              if (!(maha < merge_gate_)) continue;
              best_dist = desc_dist;
              best = j;
            }
          }
      if (best >= 0) {
        target[i] = best;
        last_update_.merged++;
      }
    }

    // (2) one feature per landmark: if several features point to a landmark,
    // the most accurate one (smallest covariance trace) updates it
    std::vector<int> observer(num_old, -1);
    for (int i = 0; i < n; ++i) {
      if (!usable[i] || target[i] < 0) continue;
      int& obs = observer[target[i]];
      if (obs < 0 || covs[i].trace() < covs[obs].trace()) obs = i;
    }
    for (int j = 0; j < num_old; ++j) {
      const int i = observer[j];
      if (i < 0) continue;
      auto& lm = landmarks_[j];
      const float response =
          (i < static_cast<int>(responses.size())) ? responses[i] : 0.0f;
      lm.num_obs++;
      lm.life_time = life_time;
      lm.descriptor = descriptors.row(i).clone();  // track appearance
      lm.response = std::max(lm.response, response);
      if (covs[i].trace() < lm.cov.trace()) {
        lm.p = points.col(i);
        lm.cov = covs[i];
      }
    }

    // (3) the other features become new landmarks
    for (int i = 0; i < n; ++i) {
      if (!usable[i] || target[i] >= 0) continue;
      const float response =
          (i < static_cast<int>(responses.size())) ? responses[i] : 0.0f;
      Landmark lm;
      lm.p = points.col(i);
      lm.cov = covs[i];
      lm.descriptor = descriptors.row(i).clone();
      lm.response = response;
      lm.life_time = life_time;
      lm.num_obs = 1;
      landmarks_.emplace_back(std::move(lm));
      last_update_.added++;
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
  }

  tactic::VertexId& vertex_id() { return vertex_id_; }
  const tactic::VertexId& vertex_id() const { return vertex_id_; }
  tactic::EdgeTransform& T_vertex_this() { return T_vertex_this_; }
  const tactic::EdgeTransform& T_vertex_this() const { return T_vertex_this_; }

 private:
  using VoxKey = pointmap::VoxKey;

  /** \brief Squared Mahalanobis gate of a duplicate */
  float merge_gate_;
  /** \brief Landmarks further than this from a feature are not compared */
  float merge_radius_;
  /** \brief Max Hamming distance to merge as the same landmark */
  float descriptor_merge_dist_;
  /** \brief Merged landmarks */
  std::vector<Landmark> landmarks_;
  /** \brief Counts of the last update */
  UpdateStats last_update_;
  /** \brief The associated vertex id (set at vertex creation) */
  tactic::VertexId vertex_id_ = tactic::VertexId::Invalid();
  /** \brief Transform from the map frame to the vertex frame */
  tactic::EdgeTransform T_vertex_this_ = tactic::EdgeTransform(true);
};

inline auto IntensityFeatureMap::fromStorable(
    const IntensityFeatureMapMsg& storable) -> Ptr {
  auto data = std::make_shared<IntensityFeatureMap>(
      storable.merge_gate, storable.merge_radius,
      storable.descriptor_merge_dist);
  const bool has_covs = storable.covariances.size() >=
                        static_cast<size_t>(storable.num_landmarks) * 9;

  const int n = static_cast<int>(storable.num_landmarks);
  const int desc_cols =
      storable.descriptor_cols > 0 ? static_cast<int>(storable.descriptor_cols)
                                   : 32;
  data->landmarks_.reserve(n);
  for (int i = 0; i < n; ++i) {
    Landmark lm;
    lm.p << storable.points[i * 3 + 0], storable.points[i * 3 + 1],
        storable.points[i * 3 + 2];
    if (has_covs)
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          lm.cov(r, c) = storable.covariances[i * 9 + r * 3 + c];
    lm.descriptor = cv::Mat(1, desc_cols, CV_8U);
    std::memcpy(lm.descriptor.data, storable.descriptors.data() + i * desc_cols,
                desc_cols);
    lm.response = storable.responses[i];
    lm.life_time = storable.life_times[i];
    lm.num_obs = storable.num_obs[i];
    data->landmarks_.emplace_back(std::move(lm));
  }

  data->vertex_id_ = tactic::VertexId(storable.vertex_id);
  using namespace vtr::common;
  conversions::fromROSMsg(storable.t_vertex_this, data->T_vertex_this_);
  return data;
}

inline auto IntensityFeatureMap::toStorable() const -> IntensityFeatureMapMsg {
  IntensityFeatureMapMsg storable;
  storable.merge_gate = merge_gate_;
  storable.merge_radius = merge_radius_;
  storable.descriptor_merge_dist = descriptor_merge_dist_;

  const int n = static_cast<int>(landmarks_.size());
  const int desc_cols =
      landmarks_.empty() ? 32 : landmarks_.front().descriptor.cols;
  storable.num_landmarks = static_cast<uint32_t>(n);
  storable.descriptor_cols = static_cast<uint32_t>(desc_cols);
  storable.points.resize(n * 3);
  storable.covariances.resize(n * 9);
  storable.descriptors.resize(n * desc_cols);
  storable.responses.resize(n);
  storable.life_times.resize(n);
  storable.num_obs.resize(n);
  for (int i = 0; i < n; ++i) {
    const auto& lm = landmarks_[i];
    storable.points[i * 3 + 0] = lm.p.x();
    storable.points[i * 3 + 1] = lm.p.y();
    storable.points[i * 3 + 2] = lm.p.z();
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        storable.covariances[i * 9 + r * 3 + c] = lm.cov(r, c);
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
