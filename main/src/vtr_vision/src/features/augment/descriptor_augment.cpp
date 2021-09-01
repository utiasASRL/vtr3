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
 * \file descriptor_augment.cpp
 * \brief Source file for the ASRL vision package
 * \details
 *
 * \author Kirk MacTavish, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_logging/logging.hpp>
#include <vtr_vision/features/augment/descriptor_augment.hpp>

namespace vtr {
namespace vision {

////////////////////////////////////////////////////////////////////
// AugmentorEngine
////////////////////////////////////////////////////////////////////

cv::Mat_<DescType> AugmentorEngine::augment(const ChannelFeatures &channel) {
  // Check the input
  if (channel.cameras.size() == 0 || channel.cameras[0].descriptors.rows == 0)
    return cv::Mat_<DescType>();
  if (channel.cameras[0].descriptors.type() != CV_32F)
    throw std::invalid_argument(
        "The descriptors must be floats or this class needs to be templated");

  // Allocate the return
  std::vector<unsigned> dims(1, 0);
  dims.reserve(augmentors_.size() + 1);
  for (const auto &aug : augmentors_)
    dims.emplace_back(dims.back() + aug->getDim(channel));
  cv::Mat_<DescType> augmented(channel.cameras[0].descriptors.rows,
                               dims.back());

  // Loop through all the augmentors
  auto offset = dims.begin();
  for (auto it = augmentors_.begin(); it != augmentors_.end(); ++it, ++offset) {
    // Augment the descriptor
    (*it)->augment(&augmented, *offset, channel);
  }

  return augmented;
}

////////////////////////////////////////////////////////////////////
// AugmentorDescriptor
////////////////////////////////////////////////////////////////////

void AugmentorDescriptor::augment(cv::Mat_<DescType> *augmented,
                                  unsigned offset,
                                  const ChannelFeatures &channel) const {
  if (channel.cameras[0].descriptors.type() != CV_32F)
    throw std::invalid_argument(
        "The descriptors must be floats or this class needs to be templated");

  // Copy the descriptor (shallow copy)
  const auto &priv_cam = channel.cameras[0];
  cv::Range cols(offset, offset + priv_cam.descriptors.cols);
  (*augmented)(cv::Range::all(), cols) =
      priv_cam.descriptors.clone() / sigma_d_;
}

////////////////////////////////////////////////////////////////////
// AugmentorDisparity
////////////////////////////////////////////////////////////////////

AugmentorDisparity::AugmentorDisparity(const RigCalibration &calibration,
                                       float sigma_d, float sigma_z)
    : calibration_(calibration), sigma_d_(sigma_d), sigma_z_(sigma_z) {
  if (sigma_d_ <= 0 || sigma_z_ <= 0)
    throw std::invalid_argument("Standard deviations must be positive");
}

unsigned AugmentorDisparity::getDim(const ChannelFeatures &channel) const {
  if (calibration_.extrinsics.size() != calibration_.intrinsics.size() ||
      calibration_.extrinsics.size() != channel.cameras.size())
    throw std::invalid_argument(
        "AugmentorDisparity: Calibration and camera sizes don't match.");
  // Only works for fully matched and rectified right now
  return channel.fully_matched && calibration_.rectified
             ? getNumCams(channel) - 1
             : 0;
}

void AugmentorDisparity::augment(cv::Mat_<DescType> *augmented, unsigned column,
                                 const ChannelFeatures &channel) const {
  if (!getDim(channel)) return;

  // rectified only
  float focal =
      calibration_.intrinsics[0](0, 0) / calibration_.intrinsics[0](2, 2);
  // center of the camera in world coordinates
  float cx0_w = -calibration_.extrinsics[0].matrix()(0, 3);

  LOG(DEBUG) << "focal: " << focal << " cx0_w: " << cx0_w << " ";

  // Loop over each camera (the first is checked and skipped
  const Features &cam0 = channel.cameras[0];
  for (unsigned j = 0; j < channel.cameras.size(); ++j) {
    const Features &camj = channel.cameras[j];

    // Make sure there are the right number of keypoints
    if (camj.keypoints.size() != (unsigned)augmented->rows)
      throw std::runtime_error(
          "Augmented descriptor has different rows than num keypoints.");

    // Disparity is relative to the privileged camera (for now, fully matched)
    if (j == 0) continue;

    float cxj_w = -calibration_.extrinsics[j].matrix()(0, 3);
    float baseline = cxj_w - cx0_w;  // usually positive
    float baseline_abs = std::abs(baseline);

    // Scaling parameters
    float a = sqrt(focal * baseline_abs / (sigma_d_ * sigma_z_));
    float c = a * sigma_d_;

    LOG(DEBUG) << "a: " << a << " c: " << c << " cxj_w: " << cxj_w
               << " base: " << baseline << " ";

    // Calculate and populate the disparity
    for (unsigned i = 0; i < camj.keypoints.size(); ++i) {
      float disparity = cam0.keypoints[i].pt.x - camj.keypoints[i].pt.x;
      disparity = std::max(0.001f, std::copysign(disparity, baseline));
      // this gives us a gradient that matches the sigmas at either extreme
      (*augmented)(i, column + j - 1) = a * (1.f - 1.f / (disparity / c + 1.f));
    }
  }
}

////////////////////////////////////////////////////////////////////
// AugmentorSize
////////////////////////////////////////////////////////////////////

void AugmentorSize::augment(cv::Mat_<DescType> *augmented, unsigned offset,
                            const ChannelFeatures &channel) const {
  // Check input
  const Keypoints &kps = channel.cameras[0].keypoints;
  if (int(kps.size()) != augmented->rows)
    throw std::out_of_range(
        "The keypoint sizes don't match the number of descriptors.");

  // Scaling parameter
  float log_sigma = log(sigma_sz_log_base_);

  // Populate each entry
  for (int i = 0; i < augmented->rows; ++i) {
    // Calculate log of size, and scale
    (*augmented)(i, offset) = log(kps[i].size) / log_sigma;
  }
}

////////////////////////////////////////////////////////////////////
// AugmentorPixelPos
////////////////////////////////////////////////////////////////////

void AugmentorPixelPos::augment(cv::Mat_<DescType> *augmented,
                                unsigned int offset,
                                const ChannelFeatures &channel) const {
  // Check input
  const Keypoints &kps = channel.cameras[0].keypoints;
  if (int(kps.size()) != augmented->rows)
    throw std::out_of_range(
        "The keypoint sizes don't match the number of descriptors.");

  // Populate each entry
  for (int i = 0; i < augmented->rows; ++i) {
    // Calculate log of size, and scale
    (*augmented)(i, offset) = kps[i].pt.x / sigma_pixel_d_;
    (*augmented)(i, offset + 1) = kps[i].pt.y / sigma_pixel_d_;
  }
}
}  // namespace vision
}  // namespace vtr
