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
 * \file gpu_surf_feature_extractor.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <mutex>

#include <vtr_vision/features/extractor/base_feature_extractor.hpp>

#include <asrl/vision/gpusurf/GpuSurfDetector.hpp>
#include <asrl/vision/gpusurf/GpuSurfStereoDetector.hpp>
#include <vtr_logging/logging.hpp>

namespace vtr {
namespace vision {

/////////////////////////////////////////////////////////////////////////
/// @class GpuSurfFeatureExtractor
/// @brief Feature extractor for the ASRL GPU Speeded Up Robust Features
///        (GPU-SURF).
/// @details This class accepts greyscale images and computes SURF features
///          on either mono images or stereo pairs with left-right matching.
/////////////////////////////////////////////////////////////////////////
class GpuSurfFeatureExtractor : public BaseFeatureExtractor {
 public:
  /////////////////////////////////////////////////////////////////////////
  /// @brief Constructor
  /// @param SURF Stereo configuration
  /////////////////////////////////////////////////////////////////////////
  GpuSurfFeatureExtractor() = default;

  /////////////////////////////////////////////////////////////////////////
  /// @brief Destructor
  /////////////////////////////////////////////////////////////////////////
  virtual ~GpuSurfFeatureExtractor(){};

  /////////////////////////////////////////////////////////////////////////
  /// @brief Initializes the underlying GPUSURF engine.
  void initialize(asrl::GpuSurfConfiguration &config);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Initializes the underlying stereo GPUSURF engine.
  void initialize(asrl::GpuSurfStereoConfiguration &config);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Extracts a list of descriptors and keypoints from a single image.
  /// @param[in] image the input image.
  /// @return the extracted features (keypoints, descriptors, info)
  virtual Features extractFeatures(const cv::Mat &image);

  /////////////////////////////////////////////////////////////////////////
  /// Extracts a list of descriptors and keypoints from a set of
  ///   two rectified stereo images.
  /// @param[in] left the left image.
  /// @param[in] right the right image.
  /// @return the extracted features that have been pruned and match-aligned.
  virtual ChannelFeatures extractStereoFeatures(const cv::Mat &left,
                                                const cv::Mat &right);

  /////////////////////////////////////////////////////////////////////////
  /// Extracts a list of descriptors and keypoints from a set of
  ///   two rectified stereo images.
  /// @param[in] left the left image.
  /// @param[in] right the right image.
  /// @return the extracted features that have been pruned and match-aligned.
  virtual ChannelFeatures extractStereoFeaturesDisp(const cv::Mat &left,
                                                    const cv::Mat &disp);

  /////////////////////////////////////////////////////////////////////////
  /// Extracts a list of descriptors and keypoints from a set of
  ///   two rectified stereo images.
  /// @param[in] left the left image.
  /// @param[in] right the right image.
  /// @return the extracted features that have been pruned and match-aligned.
  virtual ChannelFeatures extractStereoFeaturesDispExtra(const cv::Mat &left,
                                                         const cv::Mat &disp,
                                                         const cv::Mat 
                                                         &keypoints,
                                                         const cv::Mat 
                                                         &descriptors,
                                                         const cv::Mat 
                                                         &scores);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Extracts a list of descriptors and keypoints from a single image.
  /// @param[in] image the input image.
  /// @return the extracted features (keypoints, descriptors, info)
  virtual ChannelExtra extractFeaturesExtra(const cv::Mat &image);

 private:
  Features SURFToFrame(const std::vector<asrl::Keypoint> &keypoints,
                       const std::vector<float> &descriptors);

#if 0
  /////////////////////////////////////////////////////////////////////////
  /// @brief Computes the keypoints and descriptors
  /// @details In stereo mode, this function will produce a frame
  /// @param[in] The collection of input images.
  /// @param[in,out] keypoints the collection of keypoints to be populated
  /// @param[in,out] descriptors the collection of descriptors to be populated
  void computeStereoFeatures(
      std::vector<cv::Mat> &images,
      std::vector<std::vector<asrl::Keypoint>> &keypoints,
      std::vector<float> &descriptors);
#endif

  /// The SURF configuration.
  asrl::GpuSurfConfiguration config_;

  /// The SURF configuration.
  asrl::GpuSurfStereoConfiguration stereo_config_;

  /// The underlying GPU SURF detector stereo engine.
  std::unique_ptr<asrl::GpuSurfStereoDetector> stereo_detector_;

  /// The underlying GPU SURF detector engine.
  std::unique_ptr<asrl::GpuSurfDetector> detector_;

  /// The mutex that prevents multiple threads on the GPU at once.
  static std::mutex gpu_mutex_;
};

}  // namespace vision
}  // namespace vtr
