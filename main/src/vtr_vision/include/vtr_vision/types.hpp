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
 * \file types.hpp
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <array>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

// opencv definitions
#include "opencv2/core/core.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/core/version.hpp"  // defines CV_MAJOR_VERSION
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv_modules.hpp"  // defines HAVE_OPENCV_CUDAFEATURES2D

#include "lgmath.hpp"

namespace vtr {
namespace vision {

////////////////////////////////////////////////////////////////////////////////
// Calibrations

// clang-format off
using CameraDistortion = Eigen::Matrix<double, 5, 1>;
using CameraDistortions = std::vector<CameraDistortion, Eigen::aligned_allocator<CameraDistortion>>;
using CameraIntrinsic = Eigen::Matrix<double, 3, 3>;
using CameraIntrinsics = std::vector<CameraIntrinsic, Eigen::aligned_allocator<CameraIntrinsic>>;
using CameraProjection = Eigen::Matrix<double, 3, 4>;
using CameraProjections = std::vector<CameraProjection, Eigen::aligned_allocator<CameraProjection>>;
// clang-format on
using Transform = lgmath::se3::Transformation;
using Transforms = std::vector<Transform, Eigen::aligned_allocator<Transform>>;

/// Rigid camera set information.
struct RigCalibration {
  /// Distortion parameters (k0, k1, p0, p1, k3), one for each camera in rig.
  CameraDistortions distortions;
  /// Intrinsic camera matrices, one for each camera in the rig.
  CameraIntrinsics intrinsics;
  /** \brief Transform from origin point to each camera */
  Transforms extrinsics;
  /** \brief Indicates whether the rig is rectified (true) or general (false) */
  bool rectified;
};

////////////////////////////////////////////////////////////////////////////////
// Image storage

/// An image from a single channel type and camera
struct Image {
  Image() = default;

  Image(const Image &img) {
    stamp = img.stamp;
    name = img.name;
    data = img.data.clone();
  }

  Image &operator=(const Image &img) {
    stamp = img.stamp;
    name = img.name;
    data = img.data.clone();
    return *this;
  }

  Image(Image &&img) noexcept {
    stamp = img.stamp;
    name = img.name;
    data = img.data;
  }

  /// The image timestamp (ns epoch time)
  uint64_t stamp;
  /// The name of the camera (e.g. left, right)
  std::string name;
  /// The OpenCV image
  cv::Mat data;
};

/// Images from a camera rig for a single channel type
struct ChannelImages {
  ChannelImages() = default;

  ChannelImages(const ChannelImages &channel) {
    name = channel.name;
    cameras = channel.cameras;
  }

  ChannelImages &operator=(const ChannelImages &channel) {
    name = channel.name;
    cameras = channel.cameras;
    return *this;
  }

  ChannelImages(ChannelImages &&channel) noexcept {
    name = channel.name;
    for (auto &camera : channel.cameras) {
      cameras.emplace_back(std::move(camera));
    }
  }

  /// The name of the channel (e.g. grey, dessert, forest)
  std::string name;
  /// The images from the different cameras in the rig
  std::vector<Image> cameras;
};

/// Images from a camera rig
struct RigImages {
  RigImages() = default;

  RigImages(const RigImages &channel) {
    name = channel.name;
    channels = channel.channels;
  }

  RigImages &operator=(const RigImages &channel) {
    name = channel.name;
    channels = channel.channels;
    return *this;
  }

  RigImages(RigImages &&rig) noexcept {
    name = rig.name;
    for (auto &channel : rig.channels) {
      channels.emplace_back(std::move(channel));
    }
  }

  ~RigImages() = default;

  /// The name of the rig (e.g. front-xb3, rear-visensor)
  std::string name;
  /// The images from all the different channel types
  std::vector<ChannelImages> channels;
};

////////////////////////////////////////////////////////////////////////////////
// Keypoints

/// Use OpenCV's keypoint, since we need to operator on this type
using Keypoint = cv::KeyPoint;
using Keypoints = std::vector<Keypoint>;

////////////////////////////////////////////////////////////////////////////////
// Descriptors

enum struct FeatureImpl { UNKNOWN = 0, OPENCV_ORB, ASRL_GPU_SURF };
struct FeatureType {
  /// The implementation used for feature extraction
  FeatureImpl impl;
  /// The dimension of the descriptor
  unsigned dims;
  /// The number of bytes in the descriptor
  unsigned bytes_per_desc;
  /// Whether the descriptor is upright or rotated (rotation invariant)
  bool upright;
};

////////////////////////////////////////////////////////////////////////////////
// Collections of features, mirroring Features.proto but with usable types

/// Extra keypoint appearance info not covered by OpenCV's definition.
/// The split here differs from the protobuf split between keypoint and info.
struct FeatureInfo {
  /// Detector Laplacian bit
  bool laplacian_bit;
  /// Keypoint precision (1/sigma^2), where sigma is in pixels
  float precision;
  ///
  Eigen::Matrix2d covariance;
  FeatureInfo(bool lb, float pr) : laplacian_bit(lb), precision(pr) {}
  FeatureInfo() : laplacian_bit(false), precision(0.0f) {}
};
using FeatureInfos = std::vector<FeatureInfo>;

/// The collection of features from an image for a single channel type
struct Features {
  /// The camera name (e.g. left, right)
  std::string name;
  /// The keypoints detected in the image
  Keypoints keypoints;
  /// The extra keypoint information not included in the OpenCV keypoint
  FeatureInfos feat_infos;
  /// The descriptors stored as a binary blob
  cv::Mat descriptors;
  /// The descriptors stored as a binary blob
#if CV_MAJOR_VERSION >= 3 && defined(HAVE_OPENCV_CUDAFEATURES2D)
  cv::cuda::GpuMat gpu_descriptors;
#endif
  /// The type of feature extracted
  FeatureType feat_type;
};

/// The collection of features from each camera in a rig for one channel type
struct ChannelFeatures {
  /// The channel name (e.g. grey, dessert, forest)
  std::string name;
  /// The features for each camera in the rig
  std::vector<Features> cameras;
  // Matches rig_matches;
  /// Whether the features are fully matched.
  /// This means every feature has a match, and the indices are match aligned.
  bool fully_matched;
};

/// The collection of features for every camera and channel in a camera rig
struct RigFeatures {
  /// The rig name (e.g. front-xb3, rear-visensor)
  std::string name;
  /// The features for each channel in the rig
  std::vector<ChannelFeatures> channels;
};
using SuiteFeatures = std::vector<RigFeatures>;

}  // namespace vision
}  // namespace vtr
