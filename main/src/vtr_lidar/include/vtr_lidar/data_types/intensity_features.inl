/**
 * \file intensity_features.inl
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 */

 #pragma once

#include "vtr_lidar/data_types/intensity_features.hpp"

namespace vtr {
namespace lidar {

auto IntensityFeatures::fromStorable(const IntensityFeaturesMsg& msg) -> Ptr {
  auto result = std::make_shared<IntensityFeatures>();

  const int N = static_cast<int>(msg.num_features);
  result->image_width = static_cast<int>(msg.image_width);
  result->image_height = static_cast<int>(msg.image_height);

  // Keypoints
  result->keypoints.resize(N);
  for (int i = 0; i < N; ++i) {
    result->keypoints[i] = cv::KeyPoint(
        msg.keypoint_data[i * 5 + 0],   // x
        msg.keypoint_data[i * 5 + 1],   // y
        msg.keypoint_data[i * 5 + 2],   // size
        msg.keypoint_data[i * 5 + 3],   // angle
        msg.keypoint_data[i * 5 + 4],   // response
        msg.keypoint_octaves[i]          // octave
    );
  }

  // Descriptors
  if (N > 0) {
    const int desc_cols = static_cast<int>(msg.descriptor_cols);
    result->descriptors = cv::Mat(N, desc_cols, CV_8U);
    std::memcpy(result->descriptors.data, msg.descriptors.data(),
                N * desc_cols);
  }

  // 3D points
  result->points_3d.resize(3, N);
  for (int i = 0; i < N; ++i) {
    result->points_3d(0, i) = msg.points_3d[i * 3 + 0];
    result->points_3d(1, i) = msg.points_3d[i * 3 + 1];
    result->points_3d(2, i) = msg.points_3d[i * 3 + 2];
  }

  return result;
}

auto IntensityFeatures::toStorable() const -> IntensityFeaturesMsg {
  IntensityFeaturesMsg msg;

  const int N = size();
  msg.num_features = static_cast<uint32_t>(N);
  msg.image_width = static_cast<uint32_t>(image_width);
  msg.image_height = static_cast<uint32_t>(image_height);

  // Keypoints
  msg.keypoint_data.resize(N * 5);
  msg.keypoint_octaves.resize(N);
  for (int i = 0; i < N; ++i) {
    msg.keypoint_data[i * 5 + 0] = keypoints[i].pt.x;
    msg.keypoint_data[i * 5 + 1] = keypoints[i].pt.y;
    msg.keypoint_data[i * 5 + 2] = keypoints[i].size;
    msg.keypoint_data[i * 5 + 3] = keypoints[i].angle;
    msg.keypoint_data[i * 5 + 4] = keypoints[i].response;
    msg.keypoint_octaves[i] = keypoints[i].octave;
  }

  // Descriptors
  if (N > 0) {
    msg.descriptor_cols = static_cast<uint32_t>(descriptors.cols);
    msg.descriptors.resize(N * descriptors.cols);
    std::memcpy(msg.descriptors.data(), descriptors.data,
                N * descriptors.cols);
  } else {
    msg.descriptor_cols = 32;
  }

  // 3D points
  msg.points_3d.resize(N * 3);
  for (int i = 0; i < N; ++i) {
    msg.points_3d[i * 3 + 0] = points_3d(0, i);
    msg.points_3d[i * 3 + 1] = points_3d(1, i);
    msg.points_3d[i * 3 + 2] = points_3d(2, i);
  }

  return msg;
}

}  // namespace lidar
}  // namespace vtr





