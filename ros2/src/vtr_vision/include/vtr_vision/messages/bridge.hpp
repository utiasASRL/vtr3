#pragma once

#include <vtr_vision/types.hpp>

//#include <robochunk_msgs/Images.pb.h>
#include <vtr_messages/msg/image.hpp>
#include <vtr_messages/msg/channel_images.hpp>
#include <vtr_messages/msg/rig_images.hpp>

#include <robochunk_msgs/RigCalibration.pb.h>
#include <robochunk_msgs/Transform.pb.h>
//#include <robochunk_msgs/XB3CalibrationRequest.pb.h>
#include <robochunk_msgs/XB3CalibrationResponse.pb.h>
#include <asrl/messages/Descriptors.pb.h>
#include <asrl/messages/Keypoints.pb.h>
#include <asrl/messages/Matches.pb.h>
#include <asrl/messages/Pointcloud.pb.h>
#include <asrl/messages/Landmarks.pb.h>
#include <asrl/messages/Observations.pb.h>
#include <asrl/messages/BagOfWords.pb.h>
#include <asrl/messages/Features.pb.h>

// todo: file currently used to convert between robochunk and vtr data
//  structures. Will need to rewrite functions after ROS2 change.

namespace vtr {
namespace messages {

// TODO (old) make a general for_each for cameras.channel and rigs.camera
std::tuple<decltype(CV_32F),decltype(sizeof(float))>
featureCvType(const vision::FeatureImpl & type);

/// Converts the visual feature type to a string
/// @return a string representation of the visual feature type.
std::string featureType2Str(const vision::FeatureImpl &impl);

/// Converts the visual feature string to a typed enum.
/// @return an enum representation of the visual feature type.
vision::FeatureImpl str2FeatureType(std::string str);

asrl::vision_msgs::DescriptorType copyDescriptorType(const vision::FeatureType &feat_type);
vision::FeatureType copyDescriptorType(const asrl::vision_msgs::DescriptorType &desc_type);

/// Copies the keypoints from a message into a vector of OpenCV keypoints.
/// @return a vector of keypoints from the channel
vision::Features copyFeatures(
    const asrl::vision_msgs::Features & msg_features); ///<[in] the protobuf features message

/// Copies the keypoints from a message into a vector of OpenCV keypoints.
/// @return a vector^2 of keypoints from the camera
vision::ChannelFeatures copyFeatures(
    const asrl::vision_msgs::ChannelFeatures & msg_channel);

/// Copies the keypoints from a message into a vector of OpenCV keypoints.
/// @return a vector^3 of keypoints from the rig
vision::RigFeatures copyFeatures(
    const asrl::vision_msgs::RigFeatures & msg_rig);

/// Copies the contents of an vtr_vision features message into a protobuf features message
/// @return a protobuf Features Message
asrl::vision_msgs::Features copyFeatures(const vision::Features &features);

/// Copies the contents of an vtr_vision channel features message into a protobuf features message
/// @return a protobuf Features Message
asrl::vision_msgs::ChannelFeatures copyFeatures(const vision::ChannelFeatures &channel_features);

/// Copies the contents of an vtr_vision rig features message into a protobuf features message
/// @return a protobuf Features Message
asrl::vision_msgs::RigFeatures copyFeatures(const vision::RigFeatures &rig_features);

/// Copies matches from a struct into the equivalent message
/// @return a message match list that contains simple index pairs
asrl::vision_msgs::Matches copyMatches(
    const vision::LandmarkMatches & match_list); ///<[in] the match list to copy

/// Copies matches from a message into the equivalent struct
/// @return a match list that contains simple index pairs
vision::LandmarkMatches copyMatches(
    const asrl::vision_msgs::Matches &msg_match_list); ///<[in] the match list message to copy

/// Concatenates matches from two vectors of rigmatches into a single set
/// @return a concatenated vector of rigmatches
std::vector<vision::RigMatches> concatenateMatches(const std::vector<vision::RigMatches> &matches1,
    const std::vector<vision::RigMatches> &matches2);

/// Concatenates matches from two rigmatches into a single set
/// @return a concatenated rigmatches
vision::RigMatches concatenateMatches(const vision::RigMatches &matches1,
    const vision::RigMatches &matches2);

/// Copies a point cloud from a message into an Eigen Matrix
/// @return an Eigen::Matrix that wraps the point cloud data.
Eigen::Matrix<double,3,Eigen::Dynamic> copyPointCloud(
    const asrl::vision_msgs::ChannelLandmarks & landmarks); ///<[in] the landmark proto message

/// Wraps a cv::Mat around the feature descriptor WARNING can't write-protect cv::Mat
/// @return A cv::Mat that wraps the descriptor blob for this feature list.
const cv::Mat wrapDescriptors(
    const asrl::vision_msgs::Features & features); ///<[in] features with descriptor blob

/// @brief Wraps a cv::Mat around the image data in a proto message
/// @return A cv::Mat that wraps the image data in the message
cv::Mat wrapImage(
    const vtr_messages::msg::Image & asrl_image); ///<[in] the image message to copy


/// Copies an image from robochunk to vtr_vision::vision
/// @param robochunk_image The robochunk input image.
/// @return a vtr_vision:: vision version of the input image.
vision::Image copyImages(const vtr_messages::msg::Image &robochunk_image);

/// Copies an image channel from robochunk to vtr_vision::vision
/// @param robochunk_channel The robochunk input image channel.
/// @return a vtr_vision:: vision version of the input image. channel.
vision::ChannelImages copyImages(const vtr_messages::msg::ChannelImages &robochunk_channel);

/// Copies an image rig from robochunk to vtr_vision::vision
/// @param robochunk_rig The robochunk input image rig.
/// @return a vtr_vision:: vision version of the input image rig.
vision::RigImages copyImages(const vtr_messages::msg::RigImages &robochunk_rig);

/// Copies an image from vtr_vision::vision to robochunk
/// @param asrl_image The vtr_vision::vision input image.
/// @return a robochunk version of the input image.
vtr_messages::msg::Image copyImages(const vision::Image &asrl_image);

/// Copies an image channel from vtr_vision::vision to robochunk
/// @param asrl_channel The vtr_vision::vision input image channel.
/// @return an vtr_vision::vision version of the input image channel.
vtr_messages::msg::ChannelImages copyImages(const vision::ChannelImages &asrl_channel);

/// Copies an image rig from vtr_vision::vision to robochunk
/// @param asrl_rig The vtr_vision::vision input image rig.
/// @return a robochunk version of the input image rig.
vtr_messages::msg::RigImages copyImages(const vision::RigImages &asrl_rig);

/// Copies an extrinsic calibration from robochunk to vtr_vision::vision.
/// @param robochunk_extrinsics The robochunk extrinsic calibration.
/// @return an vtr_vision::vision version of the extrinsic calibration.
vision::Transform copyExtrinsics(const robochunk::kinematic_msgs::Transform  &robochunk_extrinsics);

/// Copies an intrinsic calibration from robochunk to vtr_vision::vision.
/// @param robochunk_intrinsics The robochunk intrinsic calibration.
/// @return an vtr_vision::vision version of the intrinsic calibration.
vision::CameraIntrinsic copyIntrinsics(const vtr_messages::msg::CameraCalibration &robochunk_intrinsics);

/// Copies a rig calibration from robochunk to vtr_vision::vision.
/// @param robochunk_calibration The robochunk rig calibration.
/// @return an vtr_vision::vision version of the rig calibration.
vision::RigCalibration copyCalibration(const vtr_messages::msg::RigCalibration &robochunk_calibration);

/// Copies an xb3 calibration from robochunk to vtr_vision::vision.
/// @param robochunk_calibration The robochunk xb3 calibration.
/// @return an vtr_vision::vision version of the rig calibration.
vision::RigCalibration copyCalibration(const vtr_messages::msg::XB3CalibrationResponse &robochunk_calibration);

asrl::vision_msgs::ChannelLandmarks copyLandmarks(const vision::ChannelLandmarks &asrl_landmarks);
asrl::vision_msgs::RigLandmarks copyLandmarks(const vision::RigLandmarks &asrl_landmarks);

void updateLandmarks(asrl::vision_msgs::RigLandmarks &landmarks, const vision::RigLandmarks &asrl_landmarks);
void updateLandmarks(asrl::vision_msgs::ChannelLandmarks &landmarks, const vision::ChannelLandmarks &asrl_landmarks);

vision::Observations copyObservation(const asrl::vision_msgs::Observations &robochunk_observation);
vision::ChannelObservations copyObservation(const asrl::vision_msgs::ChannelObservations &robochunk_observation);
vision::RigObservations copyObservation(const asrl::vision_msgs::RigObservations &robochunk_observation);

vision::PersistentId copyPersistentId(const asrl::graph_msgs::PersistentId & persistent_id);
asrl::graph_msgs::PersistentId copyPersistentId(const vision::PersistentId & id);

vision::LandmarkId copyLandmarkId(const asrl::vision_msgs::FeatureId &robochunk_id);
asrl::vision_msgs::FeatureId copyLandmarkId(const vision::LandmarkId &id);

vision::ChannelBowVocabulary copyChannelBowVocabulary(const asrl::vision_msgs::ChannelBowVocabulary & robochunk_channel);
asrl::vision_msgs::ChannelBowVocabulary copyChannelBowVocabulary(const vision::ChannelBowVocabulary & channel);

vision::RigBowVocabulary copyRigBowVocabulary(const asrl::vision_msgs::RigBowVocabulary & robochunk_rig);
asrl::vision_msgs::RigBowVocabulary copyRigBowVocabulary(const vision::RigBowVocabulary & rig);

vision::BowWordCount copyBowWordCount(const asrl::vision_msgs::BowWordCount & robochunk_word_count);
asrl::vision_msgs::BowWordCount copyBowWordCount(const vision::BowWordCount & robochunk_word_count);

vision::BowDescriptor copyBowDescriptor(const asrl::vision_msgs::BowDescriptor & robochunk_bow);
asrl::vision_msgs::BowDescriptor copyBowDescriptor(const vision::BowDescriptor & robochunk_bow);

}}
