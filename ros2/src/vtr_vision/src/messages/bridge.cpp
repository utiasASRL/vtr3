#include <vtr_vision/messages/bridge.hpp>
#include <asrl/common/logging.hpp>
#include <string>
#include <algorithm>
#include <cctype>
#include <type_traits>
#include <lgmath.hpp>

namespace vtr {
namespace messages {

std::tuple<decltype(CV_32F),decltype(sizeof(float))>
featureCvType(const vision::FeatureImpl & type) {
  switch (type) {
    case vision::FeatureImpl::ASRL_GPU_SURF:
      return std::make_tuple(CV_32F, sizeof(float));
    case vision::FeatureImpl::OPENCV_ORB:
      return std::make_tuple(CV_8UC1, sizeof(char));
    default:
      LOG(WARNING) << "featureCvType: Can't find the feature type "
                   << static_cast<long long>(type);
      return std::make_tuple(CV_16F,sizeof(char)); // vtr3 change: opencv 4+
  }
}

vision::FeatureImpl str2FeatureType(std::string str) {
  std::transform(str.begin(),str.end(), str.begin(), ::tolower);
  if (str.find("surf") != std::string::npos)
    return vision::FeatureImpl::ASRL_GPU_SURF;
  else if (str.find("orb") != std::string::npos)
    return vision::FeatureImpl::OPENCV_ORB;
  LOG(WARNING) << "Could not identify feature type '" << str << "'";
  return vision::FeatureImpl::UNKNOWN;
}

std::string featureType2Str(const vision::FeatureImpl &impl) {
  switch(impl) {
    case vision::FeatureImpl::ASRL_GPU_SURF:
      return "surf";
    case vision::FeatureImpl::OPENCV_ORB:
      return "orb";
    case vision::FeatureImpl::UNKNOWN:
      return "unknown";
    default:
      LOG(WARNING) << "featureType2Str: Can't find the feature type "
                   << static_cast<long long>(impl);
      return "unknown";
  }
}

vision::Features copyFeatures(const asrl::vision_msgs::Features & msg_features) {
  // name
  vision::Features features;
  features.name = msg_features.name();

  // shortcuts
  const auto & msg_keypoints = msg_features.keypoints();
  const auto & msg_kp_info = msg_features.keypoint_info();
  const auto & msg_desc_type = msg_features.desc_type();
  const auto & msg_descriptors = msg_features.descriptors();

  // descriptor type
  features.feat_type.dims = msg_desc_type.dims();
  features.feat_type.bytes_per_desc = msg_desc_type.bytes_per_desc();
  features.feat_type.upright = msg_desc_type.upright();
  features.feat_type.impl = str2FeatureType(msg_desc_type.name());

  // preallocate the number of keypoints
  int num_kps = msg_keypoints.size();
  features.keypoints.reserve(num_kps);
  features.feat_infos.reserve(num_kps);
  if(num_kps == 0) return features;

  // make sure keypoint info is as big as keypoints
  int num_info = msg_kp_info.size();
  bool use_info = num_info == num_kps;
  if (!use_info)
    LOG(WARNING) << "for " << num_kps << " keypoints, only "
                 << num_info << " info items, skipping.";

  // fill in keypoints
  for(int i = 0; i < num_kps; ++i) {
    const auto & i_pos = msg_keypoints.Get(i).position();
    auto & i_kp = features.keypoints[i];
    auto & i_info = features.feat_infos[i];

    i_kp.pt = decltype(i_kp.pt)(i_pos.x(), i_pos.y());
    if (use_info) {
      const auto & m_info = msg_kp_info.Get(i);
      i_info.laplacian_bit = m_info.laplacian_bit();
      i_kp.octave = m_info.scale();
      i_kp.angle = m_info.orientation();
      // precision isn't available in vtr_vision::vision_msgs::Features
      //i_info.precision = m_info.precision();
      i_kp.response = m_info.response();

    }
  }

  // wrap the descriptors in a cv::Mat
  const unsigned & bpd = features.feat_type.bytes_per_desc;
  decltype(sizeof(float)) byte_depth;
  decltype(CV_8UC1) cv_type;
  std::tie(cv_type, byte_depth) = featureCvType(features.feat_type.impl);
  if (bpd % byte_depth != 0) {
    LOG(ERROR) << "bytes per descriptor: " << bpd
               << " is not divisible by byte depth: " << byte_depth;
    return features;
  }
  if (num_kps * bpd != (unsigned)msg_descriptors.size()) {
    LOG(WARNING) << "the descriptor size: " << msg_descriptors.size()
                 << " is not equal to #: " << num_kps << " x B/d: " << bpd;
    return features;
  }
  features.descriptors =
      cv::Mat(num_kps, bpd/byte_depth,
              cv_type,(void*)msg_descriptors.data(),
              bpd);

  // done
  return features;
}

/// Copies the keypoints from a message into a vector of OpenCV keypoints.
/// @return a vector^2 of keypoints from the camera
vision::ChannelFeatures copyFeatures(
    const asrl::vision_msgs::ChannelFeatures & msg_channel) { ///<[in] the protobuf camera message
  vision::ChannelFeatures channel;
  channel.fully_matched = msg_channel.fully_matched();
  channel.cameras.reserve(msg_channel.cameras_size());
  for (int i=0; i < msg_channel.cameras_size(); ++i)
    channel.cameras.emplace_back(copyFeatures(msg_channel.cameras(i)));
  return channel;
}

/// Copies the keypoints from a message into a vector of OpenCV keypoints.
/// @return a vector^3 of keypoints from the rig
vision::RigFeatures copyFeatures(
    const asrl::vision_msgs::RigFeatures & msg_rig) { ///<[in] the protobuf message
  vision::RigFeatures rig;
  rig.channels.reserve(msg_rig.channels().size());
  for (int i=0; i < msg_rig.channels().size(); ++i)
    rig.channels.emplace_back(copyFeatures(msg_rig.channels(i)));
  return rig;
}

asrl::vision_msgs::ChannelFeatures copyFeatures(const vision::ChannelFeatures &channel_features) {
  asrl::vision_msgs::ChannelFeatures proto_msg;
  proto_msg.set_name(channel_features.name);

  for (auto &camera : channel_features.cameras) {
    auto *proto_camera = proto_msg.add_cameras();
    *proto_camera = copyFeatures(camera);
  }

  proto_msg.set_fully_matched(channel_features.fully_matched);
  // TODO: (old) Rig Matches
  return proto_msg;
}

asrl::vision_msgs::RigFeatures copyFeatures(const vision::RigFeatures &rig_features) {
  asrl::vision_msgs::RigFeatures proto_msg;

  proto_msg.set_name(rig_features.name);
  for (auto &channel : rig_features.channels) {
    auto *proto_channel = proto_msg.add_channels();
    *proto_channel = copyFeatures(channel);
  }
  return proto_msg;
}

asrl::vision_msgs::Matches copyMatches(const vision::LandmarkMatches &match_list) {
  asrl::vision_msgs::Matches msg_match_list;
  auto & msg_matches = *msg_match_list.mutable_matches();
  msg_matches.Reserve(match_list.size());

  for (const vision::LandmarkMatch & match : match_list) {
    asrl::vision_msgs::Match & msg_match = *msg_matches.Add();
    msg_match.mutable_from()->CopyFrom(copyLandmarkId(match.from));
    msg_match.mutable_to()->Reserve(match.to.size());
    for (const vision::LandmarkId & to : match.to) {
      msg_match.add_to()->CopyFrom(copyLandmarkId(to));
    }
  }

  return msg_match_list;
}

vision::LandmarkMatches copyMatches(const asrl::vision_msgs::Matches &msg_match_list) {
  const auto & msg_matches = msg_match_list.matches();
  vision::LandmarkMatches match_list;
  match_list.reserve(msg_matches.size());

  for (const asrl::vision_msgs::Match & msg_match : msg_matches) {
    match_list.emplace_back();
    vision::LandmarkMatch & match = match_list.back();
    if (msg_match.has_from()) match.from = copyLandmarkId(msg_match.from());
    match.to.reserve(msg_match.to_size());
    for (const asrl::vision_msgs::FeatureId & msg_to : msg_match.to()) {
      match.to.push_back(copyLandmarkId(msg_to));
    }
  }

  return match_list;
}

std::vector<vision::RigMatches> concatenateMatches(const std::vector<vision::RigMatches> &matches1, const std::vector<vision::RigMatches> &matches2) {

  // just copy matches 1 over
  std::vector<vision::RigMatches> outmatches = matches1;

  // iterate over each rig
  for(const auto & ii : matches2) {
    // find if there is a set of rig matches with the same name as the matches we are appending
    auto rigit = std::find_if(outmatches.begin(), outmatches.end(), [&](vision::RigMatches const& m){
        return m.name == ii.name;
    });

    // check if there is a rig with the same name
    if(rigit == outmatches.end()) {
      // if there are no matching rigs, just add the rig matches to the end
      outmatches.push_back(ii);
    } else {
      // if there is a matching rig, we now need to check each channel
      *rigit = concatenateMatches(*rigit, ii);
    }
  }
  return outmatches;
}

vision::RigMatches concatenateMatches(const vision::RigMatches &matches1, const vision::RigMatches &matches2) {

  // just copy matches 1 over
  vision::RigMatches outmatches = matches1;

  // iterate over each channel
  for(const auto & channel : matches2.channels) {
    // find if there is a set of channel matches with the same name as the matches we are appending
    auto channelit = std::find_if(outmatches.channels.begin(), outmatches.channels.end(), [&](vision::ChannelMatches const& m){
        return m.name == channel.name;
    });

    // check if there is a channel with the same name
    if(channelit == outmatches.channels.end()) {
      // if there are no matching channels, just add the channel matches to the end
      outmatches.channels.push_back(channel);
    } else {
      // if there are matching channels, then append the matches to the end
      channelit->matches.insert(channelit->matches.end(),channel.matches.begin(),channel.matches.end());
    }
  }
  return outmatches;
}

Eigen::Matrix<double,3,Eigen::Dynamic> copyPointCloud(const asrl::vision_msgs::ChannelLandmarks &msg_landmarks) {
  int num_points = msg_landmarks.points_size();
  if (!num_points) return Eigen::Matrix<double,3,Eigen::Dynamic>();

  //    LOG(WARNING) << "CHECK THIS FUNCTION!" << __FILE__ << " " << __LINE__;
  //    const auto proto_data = asrl_described_image.mutable_points_3d();
  //    Eigen::Map<const Eigen::Matrix<float,4,Eigen::Dynamic>,0,Eigen::OuterStride<>>
  //        homogeneous((float*)proto_data->data(),4,num_points,Eigen::OuterStride<>(proto_data->Get(0).ByteSize()));

  Eigen::Matrix<double,4,Eigen::Dynamic> homogeneous(4,num_points);
  for(int idx = 0; idx < num_points; idx++) {
    const auto & pt = msg_landmarks.points(idx);
    homogeneous.col(idx) = Eigen::Vector4d(pt.x(), pt.y(), pt.z(), pt.w());
  }

  Eigen::Matrix<double,3,Eigen::Dynamic> points
      = homogeneous.cast<double>().colwise().hnormalized();
  return points;
}

/////////////////////////////////////////////////////////////////////////////////
// @brief Wraps a cv::Mat around the feature descriptor data of a FeautreList Message.
/////////////////////////////////////////////////////////////////////////////////
const cv::Mat wrapDescriptors(const asrl::vision_msgs::Features & features) {

  // Get the descriptor type
  if (!features.has_desc_type()) return cv::Mat();
  auto type = features.desc_type();

  // Shortcut to sizes
  unsigned n = features.keypoints_size();
  unsigned bpd = type.bytes_per_desc();
  unsigned d = type.dims();

  // Check that the binary blob is the right size
  const auto & descriptor_string = features.descriptors();
  if (descriptor_string.length() != bpd * n) {
    LOG(ERROR) << "The descriptor binary blob is the wrong size: # keypoints:  " << n;
    return cv::Mat();
  }

  // Figure out the columns / type for OpenCV
  unsigned cv_type, cols;
  if(bpd == d*sizeof(float)) {
    cv_type = CV_32F;
    cols = bpd / sizeof(float);
  } else if(bpd == d*sizeof(char) || bpd*8 == d) {
    cv_type = CV_8U;
    cols = bpd / sizeof(char);
  } else {
    LOG(ERROR) << "Unknown descriptor type: " << bpd << " bytes per descriptor";
    return cv::Mat();
  }

  // Construct and return the mat around the data
  return cv::Mat(n, cols, cv_type, (void*)descriptor_string.data());
}

/////////////////////////////////////////////////////////////////////////////////
// @brief Wraps a cv::Mat around the feature descriptor data of a FeatureList Message.
/////////////////////////////////////////////////////////////////////////////////
cv::Mat wrapImage(const robochunk::sensor_msgs::Image &asrl_image) {
  const std::string & data = asrl_image.data(0);

  //assert(data != nullptr);

  // Convert to opencv
  uint32_t width = asrl_image.width();
  uint32_t height =  asrl_image.height();
  std::string encoding = asrl_image.encoding();

  if(encoding == "mono8") {
    return cv::Mat(cv::Size(width,height),CV_8UC1,(void*)data.data());
  } else if (encoding == "bgr8") {
    return  cv::Mat(cv::Size(width,height),CV_8UC3,(void*)data.data());
  } else {
    return cv::Mat();
  }
}

asrl::vision_msgs::DescriptorType copyDescriptorType(const vision::FeatureType &feat_type) {
  asrl::vision_msgs::DescriptorType proto_desc_type;
  proto_desc_type.set_name(featureType2Str(feat_type.impl));
  proto_desc_type.set_dims(feat_type.dims);
  proto_desc_type.set_bytes_per_desc(feat_type.bytes_per_desc);
  proto_desc_type.set_upright(feat_type.upright);
  return proto_desc_type;
}


vision::FeatureType copyDescriptorType(const asrl::vision_msgs::DescriptorType &desc_type) {
  vision::FeatureType feature_type;
  feature_type.impl = str2FeatureType(desc_type.name());
  feature_type.dims = desc_type.dims();
  feature_type.bytes_per_desc = desc_type.bytes_per_desc();
  feature_type.upright = desc_type.upright();
  return feature_type;
}

asrl::vision_msgs::Features copyFeatures(const vision::Features &features) {
  // name
  asrl::vision_msgs::Features proto_features;
  proto_features.set_name(features.name);

  // fill in the descriptor type
  auto *proto_desc_type = proto_features.mutable_desc_type();
  *proto_desc_type = copyDescriptorType(features.feat_type);

  // fill in the keypoint / info
  for(unsigned idx = 0; idx < features.keypoints.size(); ++idx) {
    const auto &keypoint = features.keypoints[idx];
    const auto &keypoint_info = features.feat_infos[idx];

    auto *proto_keypoint = proto_features.add_keypoints();
    auto *proto_keypoint_info = proto_features.add_keypoint_info();

    auto *position = proto_keypoint->mutable_position();
    position->set_x(keypoint.pt.x);
    position->set_y(keypoint.pt.y);
    proto_keypoint_info->set_laplacian_bit(keypoint_info.laplacian_bit);
    proto_keypoint_info->set_scale(keypoint.octave);
    proto_keypoint_info->set_orientation(keypoint.angle);
    proto_keypoint_info->set_response(keypoint.response);
    // precision isn't available in vtr_vision::vision_msgs::Features
  }

  // memcpy the descriptors over.
  auto *proto_descriptors = proto_features.mutable_descriptors();
  if (features.descriptors.step[0] != features.feat_type.bytes_per_desc)  {
    LOG(ERROR) << "feature bytes per descriptor is set incorrectly to "
               << features.feat_type.bytes_per_desc << ", should be "
               << features.descriptors.step[0];
  }
  auto datasize = features.descriptors.rows * features.descriptors.step[0];
  proto_descriptors->resize(datasize);
  memcpy(&(*proto_descriptors)[0],features.descriptors.data,datasize);

  return proto_features;
}

vision::Image copyImages(const robochunk::sensor_msgs::Image &robochunk_image) {

  vision::Image image;

  image.stamp = robochunk_image.stamp().nanoseconds_since_epoch();
  image.name = robochunk_image.name();
  image.data = wrapImage(robochunk_image).clone();

  return image;
}

vision::ChannelImages copyImages(const robochunk::sensor_msgs::ChannelImages &robochunk_channel) {
  vision::ChannelImages channel;
  channel.name = robochunk_channel.name();

  const auto& cameras = robochunk_channel.cameras();
  auto num_cameras = cameras.size();

  for(int idx = 0; idx < num_cameras; ++idx) {
    channel.cameras.emplace_back(copyImages(cameras.Get(idx)));
  }

  return channel;
}

vision::RigImages copyImages(const robochunk::sensor_msgs::RigImages &robochunk_rig) {
  vision::RigImages rig;
  rig.name = robochunk_rig.name();

  const auto& channels = robochunk_rig.channels();
  auto num_channels = channels.size();

  for(int idx = 0; idx < num_channels; ++idx) {
    auto channel = copyImages(channels.Get(idx));
    rig.channels.emplace_back(std::move(channel));
  }
  return rig;
}

robochunk::sensor_msgs::Image copyImages(const vision::Image &asrl_image) {
  robochunk::sensor_msgs::Image image;
  const auto &cv_image = asrl_image.data;
  image.mutable_stamp()->set_nanoseconds_since_epoch(asrl_image.stamp);
  image.set_name(asrl_image.name);

  image.set_width(cv_image.cols);
  image.set_height(cv_image.rows);
  image.set_step(cv_image.step);
  if(cv_image.type() == CV_8UC1) {
    image.set_encoding("mono8");
    image.set_depth(1);
  } else if(cv_image.type() == CV_8UC3) {
    image.set_encoding("bgr8");
    image.set_depth(3);
  }

  auto datasize = image.step()*image.height();
	image.add_data(&cv_image.data[0],datasize);
  return image;
}

robochunk::sensor_msgs::ChannelImages copyImages(const vision::ChannelImages &asrl_channel) {
  robochunk::sensor_msgs::ChannelImages channel;
  channel.set_name(asrl_channel.name);
  for(auto &asrl_camera : asrl_channel.cameras) {
    auto *camera = channel.add_cameras();
    *camera = copyImages(asrl_camera);
  }

  return channel;
}

robochunk::sensor_msgs::RigImages copyImages(const vision::RigImages &asrl_rig) {
  robochunk::sensor_msgs::RigImages rig;
  rig.set_name(asrl_rig.name);
  for(auto &asrl_channel : asrl_rig.channels) {
    auto *channel = rig.add_channels();
    *channel = copyImages(asrl_channel);
  }
  return rig;
}

vision::Transform copyExtrinsics(const robochunk::kinematic_msgs::Transform  &robochunk_extrinsic) {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  Eigen::Vector3d axisangle = Eigen::Vector3d(robochunk_extrinsic.orientation().x(),
                                         robochunk_extrinsic.orientation().y(),
                                         robochunk_extrinsic.orientation().z());
  transform.block(0,0,3,3) = lgmath::so3::vec2rot(axisangle);
  transform(0,3) = robochunk_extrinsic.translation().x();
  transform(1,3) = robochunk_extrinsic.translation().y();
  transform(2,3) = robochunk_extrinsic.translation().z();
  return lgmath::se3::Transformation(transform);
}

vision::CameraIntrinsic copyIntrinsics(const robochunk::sensor_msgs::CameraCalibration &robochunk_intrinsics) {
  vision::CameraIntrinsic intrinsic;
  for(int row = 0; row < 3; ++row) {
    for(int col = 0; col < 3; ++col) {
      intrinsic(row,col) = robochunk_intrinsics.k().Get(row*3+col);
    }
  }
  return intrinsic;
}

vision::RigCalibration copyCalibration(const robochunk::sensor_msgs::RigCalibration &robochunk_calibration) {
  vision::RigCalibration calibration;

  calibration.rectified = robochunk_calibration.rectified();
  auto num_cameras = robochunk_calibration.intrinsics().size();
  for(int idx = 0; idx < num_cameras; ++idx) {
    calibration.intrinsics.push_back(copyIntrinsics(robochunk_calibration.intrinsics().Get(idx)));
    calibration.extrinsics.push_back(copyExtrinsics(robochunk_calibration.extrinsics().Get(idx)));
  }
  return calibration;
}

vision::RigCalibration copyCalibration(const robochunk::sensor_msgs::XB3CalibrationResponse &robochunk_calibration) {
  vision::RigCalibration calibration;

  // the xb3 calibration is always rectified
  calibration.rectified = true;

  // fill out the extrinsics
  calibration.extrinsics.emplace_back();
  calibration.extrinsics.emplace_back();
  auto &right_extrinsics = calibration.extrinsics[1];
  Eigen::Matrix<double,4,4> right_extrinsic = Eigen::Matrix<double,4,4>::Identity();
  right_extrinsic(0,3) = -robochunk_calibration.baseline();
  right_extrinsics = lgmath::se3::Transformation(right_extrinsic);

  // fill out intrinsics
  Eigen::Matrix<double,3,3> intrinsic_matrix = Eigen::Matrix<double,3,3>::Identity();
  intrinsic_matrix(0,0) = robochunk_calibration.focallength();
  intrinsic_matrix(0,2) = robochunk_calibration.opticalcentercol();
  intrinsic_matrix(1,1) = robochunk_calibration.focallength();
  intrinsic_matrix(1,2) = robochunk_calibration.opticalcenterrow();
  calibration.intrinsics.push_back(intrinsic_matrix);
  calibration.intrinsics.push_back(intrinsic_matrix);

  return calibration;
}

asrl::vision_msgs::ChannelLandmarks copyLandmarks(const vision::ChannelLandmarks &asrl_landmarks) {
  asrl::vision_msgs::ChannelLandmarks new_landmarks;
  new_landmarks.set_name(asrl_landmarks.name);

  auto lm_info = asrl_landmarks.appearance.feat_infos.cbegin();
  for(auto kp = asrl_landmarks.appearance.keypoints.cbegin();
      kp != asrl_landmarks.appearance.keypoints.end(); ++kp, ++lm_info) {
    // copy over the feature info
    auto *proto_keypoint_info = new_landmarks.add_lm_info();
    proto_keypoint_info->set_laplacian_bit(lm_info->laplacian_bit);
    // precision isn't available in vtr_vision::vision_msgs::ChannelLandmarks
    proto_keypoint_info->set_scale(kp->octave);
    proto_keypoint_info->set_orientation(kp->angle);
    proto_keypoint_info->set_response(kp->response);
  }

  /*for(const auto &vo_obs : asrl_landmarks.vo_obs) {
    // copy over the vo observations
  } */
  for(int idx = 0; idx < asrl_landmarks.points.cols(); ++idx) {
    auto *proto_point = new_landmarks.add_points();
    auto &point = asrl_landmarks.points.col(idx);
    proto_point->set_x(point(0));
    proto_point->set_y(point(1));
    proto_point->set_z(point(2));
    proto_point->set_w(1.0);

    new_landmarks.add_num_vo_observations(1);
    auto &cov = asrl_landmarks.covariances.col(idx);
    for(int cov_idx = 0; cov_idx < 9; ++cov_idx) {
      new_landmarks.add_covariance(cov(cov_idx));
    }

    // update the validity
    new_landmarks.add_valid(asrl_landmarks.valid.at(idx));
  }

  // fill in the descriptor type
  auto *proto_desc_type = new_landmarks.mutable_desc_type();
  *proto_desc_type = copyDescriptorType(asrl_landmarks.appearance.feat_type);

  // memcpy the descriptors over.
  auto datasize = asrl_landmarks.appearance.descriptors.rows *
                  asrl_landmarks.appearance.feat_type.bytes_per_desc;
  new_landmarks.set_descriptors(asrl_landmarks.appearance.descriptors.data,datasize);
  return new_landmarks;
}

void updateLandmarks(asrl::vision_msgs::ChannelLandmarks &landmarks, const vision::ChannelLandmarks &asrl_landmarks) {

  for(int idx = 0; idx < asrl_landmarks.points.cols(); ++idx) {
    // update the landmark positions
    auto *proto_point = landmarks.mutable_points(idx);
    auto &point = asrl_landmarks.points.col(idx);
    proto_point->set_x(point(0));
    proto_point->set_y(point(1));
    proto_point->set_z(point(2));
    proto_point->set_w(1.0);

    // update the covariances
    auto &cov = asrl_landmarks.covariances.col(idx);
    for(int cov_idx = 0; cov_idx < 9; ++cov_idx) {
      // we need to index by 9 elements
      landmarks.set_covariance(idx*9+cov_idx,cov(cov_idx));
    }

    // update the validity
    landmarks.set_valid(idx, asrl_landmarks.valid.at(idx));
  }
}

asrl::vision_msgs::RigLandmarks copyLandmarks(const vision::RigLandmarks &asrl_landmarks) {
  asrl::vision_msgs::RigLandmarks landmarks;
  landmarks.set_name(asrl_landmarks.name);
  for(const auto & asrl_channel : asrl_landmarks.channels) {
    auto *channel = landmarks.add_channels();
    *channel = copyLandmarks(asrl_channel);
  }
  return landmarks;
}

void updateLandmarks(asrl::vision_msgs::RigLandmarks &landmarks, const vision::RigLandmarks &asrl_landmarks) {
  unsigned i = 0;
  for(const auto & asrl_channel : asrl_landmarks.channels) {
    auto *channel = landmarks.mutable_channels(i);
    updateLandmarks(*channel,asrl_channel);
    i++;
  }
}

vision::PersistentId copyPersistentId(const asrl::graph_msgs::PersistentId & persistent_id) {
  vision::PersistentId id;
  id.robot = persistent_id.robot();
  id.stamp = persistent_id.stamp();
  return id;
}

asrl::graph_msgs::PersistentId copyPersistentId(const vision::PersistentId & id) {
  asrl::graph_msgs::PersistentId persistent_id;
  persistent_id.set_robot(id.robot);
  persistent_id.set_stamp(id.stamp);
  return persistent_id;
}

vision::LandmarkId copyLandmarkId(const asrl::vision_msgs::FeatureId &robochunk_id) {
  vision::LandmarkId id;

  id.index = robochunk_id.idx();
  id.camera = robochunk_id.camera();
  id.channel = robochunk_id.channel();
  id.rig = robochunk_id.rig();
  id.persistent = copyPersistentId(robochunk_id.persistent());
  return id;
}

asrl::vision_msgs::FeatureId copyLandmarkId(const vision::LandmarkId &id) {
  asrl::vision_msgs::FeatureId robochunk_id;
  robochunk_id.set_idx(id.index);
  robochunk_id.set_camera(id.camera);
  robochunk_id.set_channel(id.channel);
  robochunk_id.set_rig(id.rig);
  *robochunk_id.mutable_persistent() = copyPersistentId(id.persistent);
  return robochunk_id;
}

vision::Observations copyObservation(const asrl::vision_msgs::Observations &robochunk_observation) {
  vision::Observations observations;
  observations.name = robochunk_observation.name();
  for(int kp_idx = 0; kp_idx < robochunk_observation.keypoints().size(); ++kp_idx) {

    // insert the 2D position
    const auto &robochunk_kp = robochunk_observation.keypoints().Get(kp_idx);
    observations.points.emplace_back(vision::Point(robochunk_kp.position().x(),
                                                     robochunk_kp.position().y()));

    // insert the precision
    const auto &robochunk_precision = robochunk_observation.precisions().Get(kp_idx);
    observations.precisions.emplace_back(robochunk_precision);
    // insert the covariances
    observations.covariances.emplace_back(Eigen::Matrix2d());
    auto &cov = observations.covariances.back();
    cov(0,0) = robochunk_observation.covariances().Get(kp_idx*4);
    cov(0,1) = robochunk_observation.covariances().Get(kp_idx*4+1);
    cov(1,0) = robochunk_observation.covariances().Get(kp_idx*4+2);
    cov(1,1) = robochunk_observation.covariances().Get(kp_idx*4+3);
  }

  for(int match_idx = 0; match_idx < robochunk_observation.landmarks().size(); ++match_idx) {
    const auto &robochunk_landmark = robochunk_observation.landmarks().Get(match_idx);
    observations.landmarks.emplace_back(vision::LandmarkMatch());
    auto &landmark = observations.landmarks.back();
    landmark.from = copyLandmarkId(robochunk_landmark.from());
    for(int obs_idx = 0; obs_idx < robochunk_landmark.to().size(); ++obs_idx) {
      landmark.to.push_back(copyLandmarkId(robochunk_landmark.to().Get(obs_idx)));
    }
  }
  return observations;
}

vision::ChannelObservations copyObservation(const asrl::vision_msgs::ChannelObservations &robochunk_observation) {
  vision::ChannelObservations observations;
  observations.name = robochunk_observation.name();
  for(int camera_idx = 0; camera_idx < robochunk_observation.cameras().size(); ++camera_idx) {
    observations.cameras.emplace_back(copyObservation(robochunk_observation.cameras().Get(camera_idx)));
  }
  return observations;
}

vision::RigObservations copyObservation(const asrl::vision_msgs::RigObservations &robochunk_observation) {
  vision::RigObservations observations;
  observations.name = robochunk_observation.name();
  for(int channel_idx = 0; channel_idx < robochunk_observation.channels().size(); ++channel_idx) {
    observations.channels.emplace_back(copyObservation(robochunk_observation.channels().Get(channel_idx)));
  }
  return observations;
}

vision::ChannelBowVocabulary copyChannelBowVocabulary(const asrl::vision_msgs::ChannelBowVocabulary & robochunk_channel) {
  vision::ChannelBowVocabulary channel;
  channel.reserve(robochunk_channel.words_size());
  for (const auto & cluster : robochunk_channel.words()) {
    channel.emplace_back(copyLandmarkId(cluster));
  }
  return channel;
}

asrl::vision_msgs::ChannelBowVocabulary copyChannelBowVocabulary(const vision::ChannelBowVocabulary & channel) {
  asrl::vision_msgs::ChannelBowVocabulary robochunk_vocab;
  for (const auto & word : channel) {
    *robochunk_vocab.add_words() = copyLandmarkId(word);
  }
  return robochunk_vocab;
}

asrl::vision_msgs::RigBowVocabulary copyRigBowVocabulary(const vision::RigBowVocabulary & rig) {
  asrl::vision_msgs::RigBowVocabulary robochunk_rig;
  for (const auto & channel : rig) {
    *robochunk_rig.add_channels() = copyChannelBowVocabulary(channel);
  }
  return robochunk_rig;
}

vision::RigBowVocabulary copyRigBowVocabulary(const asrl::vision_msgs::RigBowVocabulary & robochunk_rig) {
  vision::RigBowVocabulary rig;
  rig.reserve(robochunk_rig.channels_size());
  for (const auto & channel : robochunk_rig.channels()) {
    rig.emplace_back(copyChannelBowVocabulary(channel));
  }
  return rig;
}

vision::BowWordCount copyBowWordCount(const asrl::vision_msgs::BowWordCount & robochunk_word_count) {
  vision::BowWordCount word_count;
  word_count.first = copyLandmarkId(robochunk_word_count.feature());
  word_count.second = robochunk_word_count.count();
  return word_count;
}

asrl::vision_msgs::BowWordCount copyBowWordCount(const vision::BowWordCount & word_count) {
  asrl::vision_msgs::BowWordCount robochunk_word_count;
  *robochunk_word_count.mutable_feature() = copyLandmarkId(word_count.first);
  robochunk_word_count.set_count(word_count.second);
  return robochunk_word_count;
}

vision::BowDescriptor copyBowDescriptor(const asrl::vision_msgs::BowDescriptor & robochunk_bow) {
  vision::BowDescriptor bow;
  for (const auto & word_count : robochunk_bow.word_counts()) {
    bow.insert(bow.end(), copyBowWordCount(word_count)); //< .end() is optimal if list is sorted
  }
  return bow;
}

asrl::vision_msgs::BowDescriptor copyBowDescriptor(const vision::BowDescriptor & bow) {
  asrl::vision_msgs::BowDescriptor robochunk_bow;
  for (const auto & word_count : bow) {
    *robochunk_bow.add_word_counts() = copyBowWordCount(word_count);
  }
  return robochunk_bow;
}

} // namespace messages
} // namespace vtr_vision
