#include <gtest/gtest.h>

#include <vtr_vision/messages/bridge.hpp>
#include <vtr_messages/msg/descriptor_type.hpp>
#include <vtr_messages/msg/features.hpp>
#include <vtr_messages/msg/keypoint.hpp>
#include <vtr_logging/logging_init.hpp>

using namespace vtr;
namespace msgs = messages;
namespace vis = vision;
namespace vmsgs = vtr_messages::msg;

/// Verify that we can convert a ASRL rig feature message 
/// to a ROS2 rig feature message.
TEST(Vision, featuresFromROS) {

  // stereo camera rig
  vmsgs::RigFeatures rig;
  rig.name = "front";

  // grey channels
  auto grey = vmsgs::ChannelFeatures();
  grey.name = "grey";

  // left and right cameras
  auto lgrey = vmsgs::Features();
  lgrey.name = "left";
  auto rgrey = vmsgs::Features();
  rgrey.name = "right";
  grey.cameras.push_back(lgrey);
  grey.cameras.push_back(rgrey);

  // fake features
  static const unsigned n_feat = 3;
  for (auto &c : grey.cameras) {
    c.desc_type.name = "surf";
    for (unsigned i = 0; i < n_feat; ++i) {
      auto kp = vmsgs::Keypoint();
      kp.position.x = i;
      kp.position.y = i;
      c.keypoints.push_back(kp);

      auto kpi = vmsgs::FeatureInfo();
      kpi.laplacian_bit = true;
      kpi.scale = 1;
      // set orientation to pi with float precision
      kpi.orientation = (4 * std::atan(float(1)));
      // precision isn't available in vision_msgs::Features
      kpi.response = 3;
    }
  }
  rig.channels.push_back(grey);
  vis::ChannelFeatures channel = msgs::copyFeatures(grey);

  for (auto &cam : channel.cameras) {
    EXPECT_EQ(cam.feat_type.impl, vis::FeatureImpl::ASRL_GPU_SURF);
    for (unsigned i = 0; i < cam.keypoints.size(); ++i) {
      EXPECT_EQ(cam.keypoints[i].pt.x, i);
      EXPECT_EQ(cam.keypoints[i].pt.y, i);
      EXPECT_FLOAT_EQ(cam.keypoints[i].angle, 4 * std::atan(float(1)));
    }
  }
}

/// Verify that we can convert a collection of ASRL keypoints 
/// to protobuf keypoints.
void verifyKeypointConversion(const vmsgs::Features &proto_features, const vision::Features &features) {
  EXPECT_EQ(proto_features.keypoints.size(), features.keypoints.size());
  EXPECT_EQ(proto_features.keypoint_info.size(), features.feat_infos.size());

  // Make sure the keypoint and info checks out.
  for(uint32_t keypoint_idx = 0; keypoint_idx < features.keypoints.size(); ++keypoint_idx) {
    const auto &ros_keypoint = proto_features.keypoints[keypoint_idx];
    const auto &ros_keypoint_info = proto_features.keypoint_info[keypoint_idx];

    const auto &keypoint = features.keypoints[keypoint_idx];
    const auto &keypoint_info = features.feat_infos[keypoint_idx];

    EXPECT_EQ(ros_keypoint.position.x, keypoint.pt.x);
    EXPECT_EQ(ros_keypoint.position.y, keypoint.pt.y);
    EXPECT_EQ(ros_keypoint_info.laplacian_bit, keypoint_info.laplacian_bit);
    EXPECT_EQ(ros_keypoint_info.scale, keypoint.size);
    EXPECT_EQ(ros_keypoint_info.orientation, keypoint.angle);
    // precision isn't available in vision_msgs::Features
    EXPECT_EQ(ros_keypoint_info.response, keypoint.response);
  } // end check keypoints
}

/// Verify that we can convert a collection of ASRL cameras 
/// to protobuf keypoints.
void verifyCameraConversion(const vmsgs::ChannelFeatures &ros_channel,
                   const vision::ChannelFeatures &channel) {
  EXPECT_EQ(ros_channel.cameras.size(), channel.cameras.size());
  for(uint32_t cam_idx = 0; cam_idx < channel.cameras.size(); ++cam_idx) {
    const auto &proto_camera = ros_channel.cameras[cam_idx];
    const auto &camera = channel.cameras[cam_idx];

    EXPECT_EQ(proto_camera.name, camera.name);
    verifyKeypointConversion(proto_camera,camera);
    EXPECT_EQ(proto_camera.desc_type.name, messages::featureType2Str(camera.feat_type.impl));
    EXPECT_EQ(proto_camera.desc_type.dims, (int)camera.feat_type.dims);
    EXPECT_EQ(proto_camera.desc_type.bytes_per_desc, (int)camera.feat_type.bytes_per_desc);
    EXPECT_EQ(proto_camera.desc_type.upright, camera.feat_type.upright);

    // TODO (old): Verify descriptor conversion
  } // end check camera
}

/// Verify that we can convert a collection of ASRL rig features to  
/// to ROS2 rig features.
TEST(Vision, featuresToROS) {
  vis::RigFeatures rig;
  rig.name = "front_xb3";
  rig.channels.push_back(vis::ChannelFeatures());
  auto &channel = rig.channels.back();
  channel.name = "grey";
  channel.cameras.push_back(vis::Features());
  channel.cameras.push_back(vis::Features());
  auto &left = channel.cameras[0];
  auto &right = channel.cameras[1];
  left.name = "left";
  right.name = "right";

  left.feat_type.impl = vis::FeatureImpl::ASRL_GPU_SURF;
  left.feat_type.dims = 64;
  left.feat_type.bytes_per_desc = sizeof(float);
  left.feat_type.upright = true;

  right.feat_type.impl = vis::FeatureImpl::ASRL_GPU_SURF;
  right.feat_type.dims = 64;
  right.feat_type.bytes_per_desc = sizeof(float);
  right.feat_type.upright = true;

  for (uint32_t idx = 0; idx < 10; ++idx) {
    vis::Keypoint left_keypoint;
    left_keypoint.pt.x = idx;
    left_keypoint.pt.y = idx;
    left.keypoints.emplace_back(left_keypoint);
    left.feat_infos.push_back(vis::FeatureInfo(true, 0.003));

    vis::Keypoint right_keypoint;
    right_keypoint.pt.x = idx + 3;
    right_keypoint.pt.y = idx;
    right.keypoints.emplace_back(left_keypoint);
    right.feat_infos.push_back(vis::FeatureInfo(true, 0.003));
  }

  /// @brief Check to see if we can convert from ros_rig->asrl_rig
  vmsgs::RigFeatures ros_rig = msgs::copyFeatures(rig);
  EXPECT_EQ(ros_rig.name, rig.name);
  EXPECT_EQ(ros_rig.channels.size(), rig.channels.size());
  for (uint32_t idx = 0; idx < rig.channels.size(); ++idx) {
    const auto &ros_channel = ros_rig.channels[idx];
    const auto &channel = rig.channels[idx];
    EXPECT_EQ(ros_channel.name, channel.name);
    verifyCameraConversion(ros_channel, channel);
  } // end check channel
} // end Scenario
