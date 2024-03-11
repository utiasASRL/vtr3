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
 * \file bridge.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_vision/types.hpp>

#include <vtr_vision_msgs/msg/bow_descriptor.hpp>
#include <vtr_vision_msgs/msg/bow_word_count.hpp>
#include <vtr_vision_msgs/msg/camera_calibration.hpp>
#include <vtr_vision_msgs/msg/channel_bow_vocabulary.hpp>
#include <vtr_vision_msgs/msg/channel_features.hpp>
#include <vtr_vision_msgs/msg/channel_images.hpp>
#include <vtr_vision_msgs/msg/channel_landmarks.hpp>
#include <vtr_vision_msgs/msg/channel_observations.hpp>
#include <vtr_vision_msgs/msg/descriptor_type.hpp>
#include <vtr_vision_msgs/msg/feature_id.hpp>
#include <vtr_vision_msgs/msg/features.hpp>
#include <vtr_vision_msgs/msg/image.hpp>
#include <vtr_vision_msgs/msg/match.hpp>
#include <vtr_vision_msgs/msg/matches.hpp>
#include <vtr_vision_msgs/msg/observations.hpp>
#include <vtr_vision_msgs/msg/rig_bow_vocabulary.hpp>
#include <vtr_vision_msgs/msg/rig_calibration.hpp>
#include <vtr_vision_msgs/msg/rig_features.hpp>
#include <vtr_vision_msgs/msg/rig_images.hpp>
#include <vtr_vision_msgs/msg/rig_landmarks.hpp>
#include <vtr_vision_msgs/msg/rig_observations.hpp>
#include <vtr_vision_msgs/msg/run_match.hpp>
#include <vtr_vision_msgs/msg/vertex_match.hpp>
#include <vtr_vision_msgs/msg/vertex_matches.hpp>
#include <vtr_vision_msgs/msg/xb3_calibration_response.hpp>

namespace vtr {
namespace messages {

// TODO (old) make a general for_each for cameras.channel and rigs.camera
std::tuple<decltype(CV_32F), decltype(sizeof(float))> featureCvType(
    const vision::FeatureImpl &type);

/// Converts the visual feature type to a string
/// @return a string representation of the visual feature type.
std::string featureType2Str(const vision::FeatureImpl &impl);

/// Converts the visual feature string to a typed enum.
/// @return an enum representation of the visual feature type.
vision::FeatureImpl str2FeatureType(std::string str);

vtr_vision_msgs::msg::DescriptorType copyDescriptorType(
    const vision::FeatureType &feat_type);
vision::FeatureType copyDescriptorType(
    const vtr_vision_msgs::msg::DescriptorType &desc_type);

/// Copies the keypoints from a message into a vector of OpenCV keypoints.
/// @return a vector of keypoints from the channel
vision::Features copyFeatures(
    const vtr_vision_msgs::msg::Features
        &msg_features);  ///<[in] the protobuf features message

/// Copies the keypoints from a message into a vector of OpenCV keypoints.
/// @return a vector^2 of keypoints from the camera
vision::ChannelFeatures copyFeatures(
    const vtr_vision_msgs::msg::ChannelFeatures &msg_channel);

/// Copies the keypoints from a message into a vector of OpenCV keypoints.
/// @return a vector^3 of keypoints from the rig
vision::RigFeatures copyFeatures(const vtr_vision_msgs::msg::RigFeatures &msg_rig);

/// Copies the contents of an vtr_vision features message into a protobuf
/// features message
/// @return a protobuf Features Message
vtr_vision_msgs::msg::Features copyFeatures(const vision::Features &features);

/// Copies the contents of an vtr_vision channel features message into a
/// protobuf features message
/// @return a protobuf Features Message
vtr_vision_msgs::msg::ChannelFeatures copyFeatures(
    const vision::ChannelFeatures &channel_features);

/// Copies the contents of an vtr_vision rig features message into a protobuf
/// features message
/// @return a protobuf Features Message
vtr_vision_msgs::msg::RigFeatures copyFeatures(
    const vision::RigFeatures &rig_features);

/// Copies matches from a struct into the equivalent message
/// @return a message match list that contains simple index pairs
vtr_vision_msgs::msg::Matches copyMatches(
    const vision::LandmarkMatches &match_list);  ///<[in] the match list to copy

/// Copies matches from a message into the equivalent struct
/// @return a match list that contains simple index pairs
vision::LandmarkMatches copyMatches(
    const vtr_vision_msgs::msg::Matches
        &msg_match_list);  ///<[in] the match list message to copy

/// Concatenates matches from two vectors of rigmatches into a single set
/// @return a concatenated vector of rigmatches
std::vector<vision::RigMatches> concatenateMatches(
    const std::vector<vision::RigMatches> &matches1,
    const std::vector<vision::RigMatches> &matches2);

/// Concatenates matches from two rigmatches into a single set
/// @return a concatenated rigmatches
vision::RigMatches concatenateMatches(const vision::RigMatches &matches1,
                                      const vision::RigMatches &matches2);

/// @brief Wraps a cv::Mat around the image data in a proto message
/// @return A cv::Mat that wraps the image data in the message
cv::Mat wrapImage(const vtr_vision_msgs::msg::Image
                      &asrl_image);  ///<[in] the image message to copy

/// Copies an image from ROS2 to vtr_vision::vision
/// @param ros_image The ROS2 input image.
/// @return a vtr_vision:: vision version of the input image.
vision::Image copyImages(const vtr_vision_msgs::msg::Image &ros_image);

/// Copies an image channel from ROS2 to vtr_vision::vision
/// @param ros_channel The ROS2 input image channel.
/// @return a vtr_vision:: vision version of the input image. channel.
vision::ChannelImages copyImages(
    const vtr_vision_msgs::msg::ChannelImages &ros_channel);

/// Copies an image rig from ROS2 to vtr_vision::vision
/// @param ros_rig The ROS2 input image rig.
/// @return a vtr_vision:: vision version of the input image rig.
vision::RigImages copyImages(const vtr_vision_msgs::msg::RigImages &ros_rig);

/// Copies an image from vtr_vision::vision to ROS2
/// @param asrl_image The vtr_vision::vision input image.
/// @return a ROS2 version of the input image.
vtr_vision_msgs::msg::Image copyImages(const vision::Image &asrl_image);

/// Copies an image channel from vtr_vision::vision to ROS2
/// @param asrl_channel The vtr_vision::vision input image channel.
/// @return an vtr_vision::vision version of the input image channel.
vtr_vision_msgs::msg::ChannelImages copyImages(
    const vision::ChannelImages &asrl_channel);

/// Copies an image rig from vtr_vision::vision to ROS2
/// @param asrl_rig The vtr_vision::vision input image rig.
/// @return a ROS2 version of the input image rig.
vtr_vision_msgs::msg::RigImages copyImages(const vision::RigImages &asrl_rig);

/// Copies an intrinsic calibration from ROS2 to vtr_vision::vision.
/// @param ros_intrinsics The ROS2 intrinsic calibration.
/// @return an vtr_vision::vision version of the intrinsic calibration.
vision::CameraIntrinsic copyIntrinsics(
    const vtr_vision_msgs::msg::CameraCalibration &ros_intrinsics);

/// Copies a rig calibration from ROS2 to vtr_vision::vision.
/// @param ros_calibration The ROS2 rig calibration.
/// @return an vtr_vision::vision version of the rig calibration.
vision::RigCalibration copyCalibration(
    const vtr_vision_msgs::msg::RigCalibration &ros_calibration);

/// Copies an xb3 calibration from ROS2 to vtr_vision::vision.
/// @param ros_calibration The ROS2 xb3 calibration.
/// @return an vtr_vision::vision version of the rig calibration.
vision::RigCalibration copyCalibration(
    const vtr_vision_msgs::msg::XB3CalibrationResponse &ros_calibration);

vtr_vision_msgs::msg::ChannelLandmarks copyLandmarks(
    const vision::ChannelLandmarks &asrl_landmarks);
vtr_vision_msgs::msg::RigLandmarks copyLandmarks(
    const vision::RigLandmarks &asrl_landmarks);

#if 0  // todo: rewrite these functions to update Rosbag2
void updateLandmarks(vtr_vision_msgs::msg::RigLandmarks &landmarks, const vision::RigLandmarks &asrl_landmarks);
void updateLandmarks(vtr_vision_msgs::msg::ChannelLandmarks &landmarks, const vision::ChannelLandmarks &asrl_landmarks);
#endif

vision::Observations copyObservation(
    const vtr_vision_msgs::msg::Observations &ros_observation);
vision::ChannelObservations copyObservation(
    const vtr_vision_msgs::msg::ChannelObservations &ros_observation);
vision::RigObservations copyObservation(
    const vtr_vision_msgs::msg::RigObservations &ros_observation);

vision::LandmarkId copyLandmarkId(const vtr_vision_msgs::msg::FeatureId &ros_id);
vtr_vision_msgs::msg::FeatureId copyLandmarkId(const vision::LandmarkId &id);

vision::ChannelBowVocabulary copyChannelBowVocabulary(
    const vtr_vision_msgs::msg::ChannelBowVocabulary &ros_channel);
vtr_vision_msgs::msg::ChannelBowVocabulary copyChannelBowVocabulary(
    const vision::ChannelBowVocabulary &channel);

vision::RigBowVocabulary copyRigBowVocabulary(
    const vtr_vision_msgs::msg::RigBowVocabulary &ros_rig);
vtr_vision_msgs::msg::RigBowVocabulary copyRigBowVocabulary(
    const vision::RigBowVocabulary &rig);

vision::BowWordCount copyBowWordCount(
    const vtr_vision_msgs::msg::BowWordCount &ros_word_count);
vtr_vision_msgs::msg::BowWordCount copyBowWordCount(
    const vision::BowWordCount &word_count);

vision::BowDescriptor copyBowDescriptor(
    const vtr_vision_msgs::msg::BowDescriptor &ros_bow);
vtr_vision_msgs::msg::BowDescriptor copyBowDescriptor(
    const vision::BowDescriptor &bow);

}  // namespace messages
}  // namespace vtr
