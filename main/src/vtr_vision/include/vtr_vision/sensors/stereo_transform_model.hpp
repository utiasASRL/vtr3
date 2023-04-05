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
 * \file stereo_transform_model.hpp
 * \brief Header file for the ASRL vision package
 * \details
 *
 * \author Kirk MacTavish, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_vision/sensors/camera_model_interface.hpp>
#include <vtr_vision/sensors/sensor_model_base.hpp>

namespace vtr {
namespace vision {

extern template class CameraModelInterface<Eigen::Matrix3Xd, Eigen::Matrix3Xd>;

////////////////////////////////////////////////////////////////////
/// @brief This class provides a model for solving and verifying stereo camera 6
/// DOF motion.
///
/// @details
////////////////////////////////////////////////////////////////////
class StereoTransformModel
    : public SensorModelBase<Eigen::Matrix4d>,
      public CameraModelInterface<Eigen::Matrix3Xd, Eigen::Matrix3Xd> {
 public:
  // Some useful typedefs
  Eigen::Matrix4Xd x;
  typedef std::shared_ptr<StereoTransformModel> Ptr;
  typedef Eigen::Matrix<double, 3, Eigen::Dynamic> MapList;
  typedef Eigen::Matrix<double, 3, Eigen::Dynamic> PointList;
  typedef Eigen::Matrix4d SolutionType;

  /// @brief The number of points needed to satisfy the model
  static const unsigned int N = 3;

  ////////////////////////////////////////////////////////////////////
  /// @brief Constructor
  /// @param [in] inv_r The inverse measurement variance (sum of squared pixel
  /// distances)
  ////////////////////////////////////////////////////////////////////
  explicit StereoTransformModel(double inv_r = 1.0) : inv_r_(inv_r) {}

  ////////////////////////////////////////////////////////////////////
  /// @brief Set the Measurement variances
  /// @param [in] inv_r_list The inverse measurement variance (sum of squared
  /// pixel distances)
  ////////////////////////////////////////////////////////////////////
  void setMeasurementVariance(const MeasVarList& inv_r_list) {
    inv_r_list_ = inv_r_list;
  }

  ////////////////////////////////////////////////////////////////////
  // Inherited
  ////////////////////////////////////////////////////////////////////
  unsigned int getN() const { return N; }

  ////////////////////////////////////////////////////////////////////
  // Inherited
  ////////////////////////////////////////////////////////////////////
  bool solveModel(const SimpleMatches& matches, SolutionType* p_solution,
                  double threshold) const;

  ////////////////////////////////////////////////////////////////////
  // Inherited
  ////////////////////////////////////////////////////////////////////
  bool verifyMatches(const SimpleMatches& matches) const;

  ////////////////////////////////////////////////////////////////////
  /// @brief Set the camera calibration from two stereo
  /// @param [in] projection_l For the left camera
  /// @param [in] projection_r For the right camera
  ////////////////////////////////////////////////////////////////////
  void setCalibration(const CameraProjection& projection_l,
                      const CameraProjection& projection_r) {
    projection_l_ = projection_l;
    projection_r_ = projection_r;

    // Make sure the last row is proper
    Eigen::Matrix<double, 1, 4> last_row(0, 0, 1, 0);
    if (projection_l_.row(2) != last_row) {
      projection_l_.row(2) = last_row;
    }
    if (projection_r_.row(2) != last_row) {
      projection_r_.row(2) = last_row;
    }
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Set the camera projection matrices
  ////////////////////////////////////////////////////////////////////
  void setCalibration(const CameraIntrinsic& intrinsics,
                      const double baseline) {
    // setup the projection matrices from the intrinsics and baseline
    projection_l_ = intrinsics * CameraProjection::Identity();
    projection_r_ = CameraProjection::Identity();
    projection_r_(0, 3) = baseline;
    projection_r_ = intrinsics * projection_r_;

    // Make sure the last row is proper
    Eigen::Matrix<double, 1, 4> last_row(0, 0, 1, 0);
    if (projection_l_.row(2) != last_row) {
      projection_l_.row(2) = last_row;
    }
    if (projection_r_.row(2) != last_row) {
      projection_r_.row(2) = last_row;
    }
  }

  ////////////////////////////////////////////////////////////////////
  // Inherited
  ////////////////////////////////////////////////////////////////////
  virtual bool computeError(
      const SimpleMatches& matches, const SolutionType& model,
      ErrorList* whitened_errors, double* robust_error,
      double stop_error = std::numeric_limits<double>::max(),
      double sigma2 = 1.) const;

 protected:
  /// @brief The camera matrix from pinhole intrinsic calibration
  CameraProjection projection_l_;
  CameraProjection projection_r_;

  /// @brief The inverse covariance
  double inv_r_;
  MeasVarList inv_r_list_;

  // Parent members
  using CameraModelInterface::pts_query_;
  using CameraModelInterface::pts_ref_;
};

}  // namespace vision
}  // namespace vtr
