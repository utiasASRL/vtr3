/**
 * \file ouster_projector.cpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * Ported from LIVO (livo_ws/src/livo/src/projector.cpp).
 * Adapted to work with VTR's PointWithInfo and standalone config.
 */
#include "vtr_lidar/features/ouster_projector.hpp"
#include "vtr_logging/logging.hpp"

#include <algorithm>
#include <cmath>

namespace vtr {
namespace lidar {

// ── Constructor ──────────────────────────────────────────────────────────────
OusterProjector::OusterProjector(
    const OusterProjectorConfig::ConstPtr& config) {
  rows_ = config->pixels_per_column;
  cols_ = config->columns_per_frame;
  u_shift_ = config->u_shift;
  destagger_ = config->destagger;
  use_reflectivity_ = config->use_reflectivity;

  // Copy and convert elevation angles from degrees to radians
  elevation_angles_ = config->beam_altitude_angles;
  for (auto& angle : elevation_angles_) {
    angle *= M_PI / 180.0;
  }

  // Copy pixel shift LUT
  offset_lut_ = config->pixel_shift_by_row;

  // Beam offset: mm → m
  beam_offset_m_ = config->lidar_origin_to_beam_origin_mm * 1e-3;

  // Build spherical-projection intrinsics K:
  //   fx = -cols / (2π)            (azimuth, negative = right-hand)
  //   fy = -rows / |angle_range|   (elevation)
  //   cx = cols / 2,  cy = rows / 2
  const double fy =
      -static_cast<double>(rows_) /
      std::fabs(elevation_angles_.front() - elevation_angles_.back());
  const double fx = -static_cast<double>(cols_) / (2.0 * M_PI);
  const double cx = static_cast<double>(cols_) / 2.0;
  const double cy = static_cast<double>(rows_) / 2.0;

  K_ << fx, 0, cx, 0, fy, cy, 0, 0, 1;

  // Build idx_to_v_ / idx_to_u_ LUTs
  const size_t n = static_cast<size_t>(rows_) * static_cast<size_t>(cols_);
  idx_to_v_.assign(n, 0);
  idx_to_u_.assign(n, 0);

  for (int i = 0; i < rows_; ++i) {
    for (int j = 0; j < cols_; ++j) {
      size_t idx;
      if (destagger_) {
        idx = static_cast<size_t>(i) * static_cast<size_t>(cols_) +
              static_cast<size_t>(j);
      } else {
        idx = static_cast<size_t>(i) * static_cast<size_t>(cols_) +
              static_cast<size_t>((j + cols_ - offset_lut_[i]) % cols_);
      }
      idx_to_v_[idx] = i;
      int col = j - u_shift_;
      if (col < 0) col += cols_;
      if (col >= cols_) col -= cols_;
      idx_to_u_[idx] = col;
    }
  }

  CLOG(INFO, "lidar.ouster_projector")
      << "OusterProjector initialized: " << rows_ << "x" << cols_
      << "  beam_offset=" << beam_offset_m_ << "m"
      << "  K=diag(" << K_(0, 0) << ", " << K_(1, 1) << ")";
}

// ── createImages ─────────────────────────────────────────────────────────────
void OusterProjector::createImages(
    const pcl::PointCloud<PointWithInfo>& cloud, cv::Mat& intensity_image,
    cv::Mat& range_image, cv::Mat& pixel_to_point_index) const {
  intensity_image = cv::Mat::zeros(rows_, cols_, CV_32FC1);
  range_image = cv::Mat::zeros(rows_, cols_, CV_32FC1);
  pixel_to_point_index = cv::Mat::ones(rows_, cols_, CV_32SC1) * (-1);

  const size_t n = cloud.size();
  const size_t expected = static_cast<size_t>(rows_) * static_cast<size_t>(cols_);

  if (n != expected) {
    CLOG(WARNING, "lidar.ouster_projector")
        << "Cloud size (" << n << ") != expected (" << expected
        << "). Falling back to spherical projection.";
    // Fallback: use analytical spherical projection
    for (size_t i = 0; i < n; ++i) {
      const auto& pt = cloud[i];
      const float x = pt.x, y = pt.y, z = pt.z;

      const bool valid =
          std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
      if (!valid) continue;

      const float range = std::sqrt(x * x + y * y + z * z);
      if (range < 0.1f) continue;

      Eigen::Vector3d point(x, y, z);
      Eigen::Vector2d uv;
      if (!projectPoint(point, uv)) continue;

      const int row = static_cast<int>(std::round(uv.y()));
      const int col = static_cast<int>(std::round(uv.x()));
      if (row < 0 || row >= rows_ || col < 0 || col >= cols_) continue;

      // Keep closest point per pixel
      if (pixel_to_point_index.at<int>(row, col) == -1 ||
          range < range_image.at<float>(row, col)) {
        intensity_image.at<float>(row, col) = pt.intensity;
        range_image.at<float>(row, col) = range;
        pixel_to_point_index.at<int>(row, col) = static_cast<int>(i);
      }
    }
    return;
  }

  // Primary path: use hardware-index LUT (exact, no trig)
  for (size_t i = 0; i < n; ++i) {
    const auto& pt = cloud[i];
    const int v = idx_to_v_[i];
    const int u = idx_to_u_[i];

    const bool valid =
        std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z);

    // Use intensity field from PointWithInfo
    const float raw = pt.intensity;

    intensity_image.ptr<float>(v)[u] =
        (valid && std::isfinite(raw)) ? raw : 0.0f;

    if (valid) {
      range_image.ptr<float>(v)[u] =
          std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      pixel_to_point_index.ptr<int>(v)[u] = static_cast<int>(i);
    }
  }
}

// ── projectPoint ─────────────────────────────────────────────────────────────
bool OusterProjector::projectPoint(const Eigen::Vector3d& point, Eigen::Vector2d& uv) const {
  // Beam-offset spherical model (COIN-LIO):
  //   L = sqrt(x²+y²) - beam_offset
  //   R = sqrt(z² + L²)
  const double L =
      std::sqrt(point.x() * point.x() + point.y() * point.y()) -
      beam_offset_m_;
  const double R = std::sqrt(point.z() * point.z() + L * L);
  const double phi = std::atan2(point.y(), point.x());
  const double theta = std::asin(std::clamp(point.z() / R, -1.0, 1.0));

  uv.x() = K_(0, 0) * phi + K_(0, 2);

  // Elevation angle look-up + sub-pixel interpolation
  // elevation_angles_ is in descending order (top beam first)
  if (theta > elevation_angles_.front()) {
    uv.y() = 0;
    return false;
  }
  if (theta < elevation_angles_.back()) {
    uv.y() = static_cast<double>(rows_) - 1.0;
    return false;
  }

  auto greater =
      (std::upper_bound(elevation_angles_.rbegin(), elevation_angles_.rend(),
                        theta) +
       1)
          .base();
  auto smaller = greater + 1;

  if (greater == elevation_angles_.end()) {
    uv.y() = static_cast<double>(rows_) - 1.0;
  } else {
    uv.y() = static_cast<double>(
        std::distance(elevation_angles_.begin(), greater));
    uv.y() += (*greater - theta) / (*greater - *smaller);
  }

  return isFOV(uv);
}

// ── isFOV ────────────────────────────────────────────────────────────────────
bool OusterProjector::isFOV(const Eigen::Vector2d& uv) const {
  return uv.x() >= 0.0 &&
         uv.x() <= static_cast<double>(cols_) - 1.0 &&
         uv.y() >= 0.0 &&
         uv.y() <= static_cast<double>(rows_) - 1.0;
}

// ── projectionJacobian ───────────────────────────────────────────────────────
void OusterProjector::projectionJacobian(
    const Eigen::Vector3d& p, Eigen::Matrix<double, 2, 3>& du_dp) const {
  const double rxy = p.head<2>().norm();
  const double L = rxy - beam_offset_m_;
  const double R2 = L * L + p.z() * p.z();
  const double irxy = 1.0 / rxy;
  const double irxy2 = irxy * irxy;
  const double fx_i2 = K_(0, 0) * irxy2;

  // Row 0:  ∂u/∂p  where u = fx·atan2(y,x) + cx
  // Row 1:  ∂v/∂p  where v = fy·asin(z/R) + cy,  R=√(L²+z²),  L=rxy−b
  du_dp << -fx_i2 * p.y(), fx_i2 * p.x(), 0.0,
      -K_(1, 1) * p.x() * p.z() * irxy / R2,
      -K_(1, 1) * p.y() * p.z() * irxy / R2, K_(1, 1) * L / R2;
}

// ── backProject ──────────────────────────────────────────────────────────────
Eigen::Vector3d OusterProjector::backProject(const Eigen::Vector2d& uv, double range) const {
  // Recover azimuth and elevation from pixel coordinates
  const double phi = (uv.x() - K_(0, 2)) / K_(0, 0);

  // Elevation: interpolate from the LUT row
  const double row_f = uv.y();
  const int row0 = std::clamp(static_cast<int>(row_f), 0, rows_ - 1);
  const int row1 = std::min(row0 + 1, rows_ - 1);
  const double alpha = row_f - row0;
  const double theta =
      (1.0 - alpha) * elevation_angles_[row0] +
      alpha * elevation_angles_[row1];

  // Reconstruct 3-D point:
  //   R = range,  z = R·sin(θ),  L = R·cos(θ),  rxy = L + offset
  const double z = range * std::sin(theta);
  const double L = range * std::cos(theta);
  const double rxy = L + beam_offset_m_;
  const double x = rxy * std::cos(phi);
  const double y = rxy * std::sin(phi);

  return Eigen::Vector3d(x, y, z);
}

}  // namespace lidar
}  // namespace vtr
