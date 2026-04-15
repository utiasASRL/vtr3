/**
 * \file ouster_projector.hpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * Spherical projection of an Ouster LiDAR point cloud into dense 2-D images.
 * Accounts for the Ouster beam offset (COIN-LIO model).
 *
 * Ported from LIVO (livo_ws/src/livo/include/projector.hpp).
 * Adapted to work with VTR's PointWithInfo and standalone config (no ROS node
 * dependency for construction).
 *
 * Original source:
 * [1] COIN-LIO beam-offset model
 * [2] Ouster SDK stagger/destagger conventions
 */
#pragma once

#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <pcl/point_cloud.h>

#include "vtr_common/utils/macros.hpp"
#include "vtr_lidar/data_types/point.hpp"

namespace vtr {
namespace lidar {

/// Configuration for the Ouster spherical projector.
/// These values come from the Ouster sensor metadata JSON.
struct OusterProjectorConfig {
  PTR_TYPEDEFS(OusterProjectorConfig);

  /// Number of beams (rows) — e.g. 128 for OS1-128
  int pixels_per_column = 128;
  /// Number of columns per revolution — e.g. 1024 at 10Hz
  int columns_per_frame = 1024;
  /// Beam origin offset from lidar origin [mm] (will be converted to m)
  double lidar_origin_to_beam_origin_mm = 15.806;
  /// Per-row pixel shift for destaggering
  std::vector<int> pixel_shift_by_row;
  /// Beam elevation angles in degrees (descending order, top beam first)
  std::vector<double> beam_altitude_angles;

  /// Horizontal pixel shift (azimuth fine-tune)
  int u_shift = 0;
  /// Whether the Ouster driver already destaggered the cloud
  bool destagger = true;
  /// Whether to use reflectivity instead of intensity
  bool use_reflectivity = false;
};

/**
 * \brief Ouster spherical projector for creating intensity/range images
 *        from LiDAR point clouds using hardware index LUTs.
 *
 * This is more accurate than generic azimuth/elevation projection because
 * it uses the exact beam angles and stagger offsets from the sensor metadata.
 */
class OusterProjector {
 public:
  PTR_TYPEDEFS(OusterProjector);

  /// Construct from config (no ROS node dependency).
  explicit OusterProjector(const OusterProjectorConfig::ConstPtr& config);

  // ── Image generation ───────────────────────────────────────────────────

  /// Create intensity, range, and index images from a PointWithInfo cloud.
  /// Uses hardware-index LUT (same as LIVO Projector::createImages).
  ///
  /// @param cloud           Input point cloud (must be structured: rows*cols)
  /// @param intensity_image Output: CV_32FC1 raw intensity values
  /// @param range_image     Output: CV_32FC1 range in metres
  /// @param pixel_to_point_index  Output: CV_32SC1 per-pixel cloud index (-1=invalid)
  void createImages(const pcl::PointCloud<PointWithInfo>& cloud,
                    cv::Mat& intensity_image,
                    cv::Mat& range_image,
                    cv::Mat& pixel_to_point_index) const;

  // ── Projection helpers ─────────────────────────────────────────────────

  /// Project a 3-D point to (u, v) image coordinates.
  /// Returns false when the point is outside the sensor FOV.
  bool projectPoint(const Eigen::Vector3d& point, Eigen::Vector2d& uv) const;

  /// True if uv lies inside the image boundaries.
  bool isFOV(const Eigen::Vector2d& uv) const;

  /// 2×3 projection Jacobian ∂(u,v)/∂p (beam-offset spherical model).
  void projectionJacobian(const Eigen::Vector3d& p,
                          Eigen::Matrix<double, 2, 3>& du_dp) const;

  /// Back-project a pixel (u, v) + range to a 3-D point.
  Eigen::Vector3d backProject(const Eigen::Vector2d& uv, double range) const;

  // ── Accessors ──────────────────────────────────────────────────────────
  int rows() const { return rows_; }
  int cols() const { return cols_; }
  double beamOffsetM() const { return beam_offset_m_; }
  const Eigen::Matrix3d& K() const { return K_; }
  const std::vector<double>& elevationAngles() const {
    return elevation_angles_;
  }

 private:
  // ── Sensor geometry ────────────────────────────────────────────────────
  double beam_offset_m_ = 0.0;
  int u_shift_ = 0;
  int rows_ = 0;
  int cols_ = 0;
  bool destagger_ = true;
  bool use_reflectivity_ = false;

  std::vector<int> offset_lut_;
  std::vector<double> elevation_angles_;  // radians, descending order

  std::vector<int> idx_to_v_;
  std::vector<int> idx_to_u_;

  Eigen::Matrix3d K_;  // Intrinsic-like matrix for spherical projection
};

}  // namespace lidar
}  // namespace vtr
