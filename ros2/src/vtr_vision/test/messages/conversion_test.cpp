#include <catch.hpp>
#include <asrl/vision/messages/bridge.hpp>
#include <asrl/messages/vision_msgs.hpp>
#include <asrl/common/math/constants.hpp>

using namespace asrl;
namespace msgs = messages;
namespace vis = vision;
namespace vmsgs = vision_msgs;

/// Verify that we can convert a ASRL rig feature message 
/// to a Protobuf rig feature message.
SCENARIO("we can convert features","[vision][conversion]") {
  GIVEN("a features proto message"){
    // stereo camera rig
    vmsgs::RigFeatures rig;
    rig.set_name("front");

    // grey channels
    auto & grey = *rig.add_channels();
    grey.set_name("grey");

    // left and right cameras
    auto & lgrey = *grey.add_cameras();
    lgrey.set_name("left");
    auto & rgrey = *grey.add_cameras();
    rgrey.set_name("right");

    // fake features
    static const unsigned n_feat = 3;
    for (auto & c : *grey.mutable_cameras()) {
      c.mutable_desc_type()->set_name("surf");
      for (unsigned i=0; i<n_feat; ++i) {
        auto & kp = *c.add_keypoints();
        auto & pos = *kp.mutable_position();
        pos.set_x(i);
        pos.set_y(i);
        auto & kpi = *c.add_keypoint_info();
        kpi.set_laplacian_bit(true);
        kpi.set_scale(1);
        kpi.set_orientation(common::pi<float>());
        // precision isn't available in vision_msgs::Features
        kpi.set_response(3);
      }
    }

    WHEN("we convert the keypoints") {
      vis::ChannelFeatures channel = msgs::copyFeatures(grey);

      THEN("we get back the original positions") {
        for (auto & cam : channel.cameras) {
          REQUIRE(cam.feat_type.impl == vis::FeatureImpl::ASRL_GPU_SURF);
          for (unsigned i=0; i<cam.keypoints.size(); ++i) {
            REQUIRE(cam.keypoints[i].pt.x == i);
            REQUIRE(cam.keypoints[i].pt.y == i);
            REQUIRE(cam.keypoints[i].angle == common::pi<float>());
          }
        }
      } // THEN
    } // WHEN
  } // GIVEN
} // SCENARIO

/// Verify that we can convert a collection of ASRL keypoints 
/// to protobuf keypoints.
void verifyKeypointConversion(const vision_msgs::Features &proto_features, const vision::Features &features) {
  REQUIRE(proto_features.keypoints().size() == features.keypoints.size());
  REQUIRE(proto_features.keypoint_info().size() == features.feat_infos.size());

  // Make sure the keypoint and info checks out.
  for(uint32_t keypoint_idx = 0; keypoint_idx < features.keypoints.size(); ++keypoint_idx) {
    const auto &proto_keypoint = proto_features.keypoints().Get(keypoint_idx);
    const auto &proto_keypoint_info = proto_features.keypoint_info().Get(keypoint_idx);

    const auto &keypoint = features.keypoints[keypoint_idx];
    const auto &keypoint_info = features.feat_infos[keypoint_idx];

    CHECK(proto_keypoint.position().x() == keypoint.pt.x);
    CHECK(proto_keypoint.position().y() == keypoint.pt.y);
    CHECK(proto_keypoint_info.laplacian_bit() == keypoint_info.laplacian_bit);
    CHECK(proto_keypoint_info.scale() == keypoint.size);
    CHECK(proto_keypoint_info.orientation() == keypoint.angle);
    // precision isn't available in vision_msgs::Features
    CHECK(proto_keypoint_info.response() == keypoint.response);
  } // end check keypoints
}

/// Verify that we can convert a collection of ASRL cameras 
/// to protobuf keypoints.
void verifyCameraConversion(const vision_msgs::ChannelFeatures &proto_channel, 
                   const vision::ChannelFeatures &channel) {
  REQUIRE(proto_channel.cameras().size() == channel.cameras.size());
  for(uint32_t cam_idx = 0; cam_idx < channel.cameras.size(); ++cam_idx) {
    const auto &proto_camera = proto_channel.cameras().Get(cam_idx);
    const auto &camera = channel.cameras[cam_idx];

    CHECK(proto_camera.name() == camera.name);
    verifyKeypointConversion(proto_camera,camera);
    CHECK(proto_camera.desc_type().name() == messages::featureType2Str(camera.feat_type.impl));
    CHECK(proto_camera.desc_type().dims() == (int)camera.feat_type.dims);
    CHECK(proto_camera.desc_type().bytes_per_desc() == (int)camera.feat_type.bytes_per_desc);
    CHECK(proto_camera.desc_type().upright() == camera.feat_type.upright);

    // TODO: Verify descriptor conversion
  } // end check camera
}

/// Verify that we can convert a collection of ASRL rig features to  
/// to protobuf rig features.
SCENARIO("We can convert Rig features") {
  GIVEN("An asrl Rig features message") {
    vis::RigFeatures rig;
    rig.name = "front_xb3";
    rig.channels.push_back(vis::ChannelFeatures());
    auto &channel = rig.channels.back();
    channel.name="grey";
    channel.cameras.push_back(vis::Features());
    channel.cameras.push_back(vis::Features());
    auto &left = channel.cameras[0];
    auto &right = channel.cameras[1];
    left.name="left";
    right.name="right";

    left.feat_type.impl = vis::FeatureImpl::ASRL_GPU_SURF;
    left.feat_type.dims = 64;
    left.feat_type.bytes_per_desc = sizeof(float);
    left.feat_type.upright = true;

    right.feat_type.impl = vis::FeatureImpl::ASRL_GPU_SURF;
    right.feat_type.dims = 64;
    right.feat_type.bytes_per_desc = sizeof(float);
    right.feat_type.upright = true;

    for(uint32_t idx = 0; idx < 10; ++idx) {
      vis::Keypoint left_keypoint;
      left_keypoint.pt.x = idx;
      left_keypoint.pt.y = idx;
      left.keypoints.emplace_back(left_keypoint);
      left.feat_infos.push_back(vis::FeatureInfo(true,0.003));

      vis::Keypoint right_keypoint;
      right_keypoint.pt.x = idx+3;
      right_keypoint.pt.y = idx;
      right.keypoints.emplace_back(left_keypoint);
      right.feat_infos.push_back(vis::FeatureInfo(true,0.003));
    }

    /// @brief Check to see if we can convert from proto_rig->asrl_rig
    WHEN("We convert the asrl channel features") {
      vmsgs::RigFeatures proto_rig = msgs::copyFeatures(rig);
     THEN("We get back the original positions") {
        CHECK(proto_rig.name() == rig.name);
        REQUIRE(proto_rig.channels().size() == rig.channels.size());
        for(uint32_t idx = 0; idx < rig.channels.size(); ++idx) {
          const auto &proto_channel = proto_rig.channels().Get(idx);
          const auto &channel = rig.channels[idx];
          CHECK(proto_channel.name() == channel.name);
          verifyCameraConversion(proto_channel,channel);
        } // end check channel
      } // end check rig
    } // end When
  } // end Given
} // end Scenario
