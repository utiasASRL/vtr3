#include <vtr_vision/messages/bridge.hpp>
#include <vtr_logging/logging.hpp>
#include <string>
#include <algorithm>
#include <cctype>
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
      return std::make_tuple(CV_16F,sizeof(char));
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

vision::Features copyFeatures(const vtr_messages::msg::Features & msg_features) {
  // name
  vision::Features features;
  features.name = msg_features.name;

  // shortcuts
  const auto & msg_keypoints = msg_features.keypoints;
  const auto & msg_kp_info = msg_features.keypoint_info;
  const auto & msg_desc_type = msg_features.desc_type;
  const auto & msg_descriptors = msg_features.descriptors;

  // descriptor type
  features.feat_type.dims = msg_desc_type.dims;
  features.feat_type.bytes_per_desc = msg_desc_type.bytes_per_desc;
  features.feat_type.upright = msg_desc_type.upright;
  features.feat_type.impl = str2FeatureType(msg_desc_type.name);

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
    const auto & i_pos = msg_keypoints[i].position;
    auto & i_kp = features.keypoints[i];
    auto & i_info = features.feat_infos[i];

    i_kp.pt = decltype(i_kp.pt)(i_pos.x, i_pos.y);
    if (use_info) {
      const auto & m_info = msg_kp_info[i];
      i_info.laplacian_bit = m_info.laplacian_bit;
      i_kp.octave = m_info.scale;
      i_kp.angle = m_info.orientation;
      // precision isn't available in vtr_vision::vision_msgs::Features
      //i_info.precision = m_info.precision();
      i_kp.response = m_info.response;
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

vision::ChannelFeatures copyFeatures(
    const vtr_messages::msg::ChannelFeatures & msg_channel) { ///<[in] the protobuf camera message
  vision::ChannelFeatures channel;
  channel.fully_matched = msg_channel.fully_matched;
  channel.cameras.reserve(msg_channel.cameras.size());
  for (const auto & camera : msg_channel.cameras)
    channel.cameras.emplace_back(copyFeatures(camera));
  return channel;
}

vision::RigFeatures copyFeatures(
    const vtr_messages::msg::RigFeatures & msg_rig) { ///<[in] the protobuf message
  vision::RigFeatures rig;
  rig.channels.reserve(msg_rig.channels.size());
  for (const auto & channel : msg_rig.channels)
    rig.channels.emplace_back(copyFeatures(channel));
  return rig;
}

vtr_messages::msg::ChannelFeatures copyFeatures(const vision::ChannelFeatures &channel_features) {
  vtr_messages::msg::ChannelFeatures ros_msg;
  ros_msg.name = channel_features.name;

  for (auto &camera : channel_features.cameras) {
    ros_msg.cameras.push_back(copyFeatures(camera));
  }

  ros_msg.fully_matched = channel_features.fully_matched;
  // TODO: (old) Rig Matches
  return ros_msg;
}

vtr_messages::msg::RigFeatures copyFeatures(const vision::RigFeatures &rig_features) {
  vtr_messages::msg::RigFeatures ros_msg;

  ros_msg.name = rig_features.name;
  for (auto &channel : rig_features.channels) {
    ros_msg.channels.push_back(copyFeatures(channel));
  }
  return ros_msg;
}

vtr_messages::msg::Matches copyMatches(const vision::LandmarkMatches &match_list) {
  vtr_messages::msg::Matches msg_match_list;
  auto & msg_matches = msg_match_list.matches;
  msg_matches.reserve(match_list.size());

  for (const vision::LandmarkMatch & match : match_list) {
    vtr_messages::msg::Match msg_match;// = *msg_matches.Add();
    msg_match.from_id = copyLandmarkId(match.from);
    msg_match.to_id.reserve(match.to.size());
    for (const vision::LandmarkId & to : match.to) {
      msg_match.to_id.push_back(copyLandmarkId(to));
    }
    msg_matches.push_back(msg_match);
  }

  return msg_match_list;
}

vision::LandmarkMatches copyMatches(const vtr_messages::msg::Matches &msg_match_list) {
  const auto & msg_matches = msg_match_list.matches;
  vision::LandmarkMatches match_list;
  match_list.reserve(msg_matches.size());

  for (const vtr_messages::msg::Match & msg_match : msg_matches) {
    match_list.emplace_back();
    vision::LandmarkMatch & match = match_list.back();
    //Check that from_id has been assigned
    if (msg_match.from_id != vtr_messages::msg::FeatureId()){
      match.from = copyLandmarkId(msg_match.from_id);
    }
    match.to.reserve(msg_match.to_id.size());
    for (const vtr_messages::msg::FeatureId & msg_to : msg_match.to_id) {
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

Eigen::Matrix<double,3,Eigen::Dynamic> copyPointCloud(const vtr_messages::msg::ChannelLandmarks &msg_landmarks) {
  int num_points = msg_landmarks.points.size();
  if (!num_points) return Eigen::Matrix<double,3,Eigen::Dynamic>();

  //    LOG(WARNING) << "CHECK THIS FUNCTION!" << __FILE__ << " " << __LINE__;
  //    const auto proto_data = asrl_described_image.mutable_points_3d();
  //    Eigen::Map<const Eigen::Matrix<float,4,Eigen::Dynamic>,0,Eigen::OuterStride<>>
  //        homogeneous((float*)proto_data->data(),4,num_points,Eigen::OuterStride<>(proto_data->Get(0).ByteSize()));

  Eigen::Matrix<double,4,Eigen::Dynamic> homogeneous(4,num_points);
  for(int idx = 0; idx < num_points; idx++) {
    const auto & pt = msg_landmarks.points[idx];
    homogeneous.col(idx) = Eigen::Vector4d(pt.x, pt.y, pt.z, pt.w);
  }

  Eigen::Matrix<double,3,Eigen::Dynamic> points
      = homogeneous.cast<double>().colwise().hnormalized();
  return points;
}

cv::Mat wrapDescriptors(const vtr_messages::msg::Features & features) {

  // Get the descriptor type
  if (features.desc_type == vtr_messages::msg::DescriptorType()) return cv::Mat();
  auto type = features.desc_type;

  // Shortcut to sizes
  unsigned n = features.keypoints.size();
  unsigned bpd = type.bytes_per_desc;
  unsigned d = type.dims;

  // Check that the binary blob is the right size
  const auto & descriptor_string = features.descriptors;
  if (descriptor_string.size() != bpd * n) {
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

cv::Mat wrapImage(const vtr_messages::msg::Image &asrl_image) {
  const auto & data = asrl_image.data;

  //assert(data != nullptr);

  // Convert to opencv
  uint32_t width = asrl_image.width;
  uint32_t height =  asrl_image.height;
  std::string encoding = asrl_image.encoding;

  if(encoding == "mono8") {
    return cv::Mat(cv::Size(width,height),CV_8UC1,(void*)data.data());
  } else if (encoding == "bgr8") {
    return  cv::Mat(cv::Size(width,height),CV_8UC3,(void*)data.data());
  } else {
    return cv::Mat();
  }
}

vtr_messages::msg::DescriptorType copyDescriptorType(const vision::FeatureType &feat_type) {
  vtr_messages::msg::DescriptorType ros_desc_type;
  ros_desc_type.name = featureType2Str(feat_type.impl);
  ros_desc_type.dims = feat_type.dims;
  ros_desc_type.bytes_per_desc = feat_type.bytes_per_desc;
  ros_desc_type.upright = feat_type.upright;
  return ros_desc_type;
}

vision::FeatureType copyDescriptorType(const vtr_messages::msg::DescriptorType &desc_type) {
  vision::FeatureType feature_type;
  feature_type.impl = str2FeatureType(desc_type.name);
  feature_type.dims = desc_type.dims;
  feature_type.bytes_per_desc = desc_type.bytes_per_desc;
  feature_type.upright = desc_type.upright;
  return feature_type;
}

vtr_messages::msg::Features copyFeatures(const vision::Features &features) {
  // name
  vtr_messages::msg::Features ros_features;
  ros_features.name = features.name;

  // fill in the descriptor type
  ros_features.desc_type = copyDescriptorType(features.feat_type);

  // fill in the keypoint / info
  for(unsigned idx = 0; idx < features.keypoints.size(); ++idx) {
    const auto &keypoint = features.keypoints[idx];
    const auto &keypoint_info = features.feat_infos[idx];

    vtr_messages::msg::Keypoint ros_keypoint;
    ros_keypoint.position.x = keypoint.pt.x;
    ros_keypoint.position.y = keypoint.pt.y;

    vtr_messages::msg::FeatureInfo ros_keypoint_info;
    ros_keypoint_info.laplacian_bit = keypoint_info.laplacian_bit;
    ros_keypoint_info.scale = keypoint.octave;
    ros_keypoint_info.orientation = keypoint.angle;
    ros_keypoint_info.response = keypoint.response;
    // precision isn't available in vtr_vision::vision_msgs::Features

    ros_features.keypoints.push_back(ros_keypoint);
    ros_features.keypoint_info.push_back(ros_keypoint_info);
  }

  // memcpy the descriptors over.
  auto ros_descriptors = &ros_features.descriptors;
  if (features.descriptors.step[0] != features.feat_type.bytes_per_desc)  {
    LOG(ERROR) << "feature bytes per descriptor is set incorrectly to "
               << features.feat_type.bytes_per_desc << ", should be "
               << features.descriptors.step[0];
  }
  auto datasize = features.descriptors.rows * features.descriptors.step[0];
  ros_descriptors->resize(datasize);
  memcpy(&(*ros_descriptors)[0],features.descriptors.data,datasize);

  return ros_features;
}

vision::Image copyImages(const vtr_messages::msg::Image &ros_image) {

  vision::Image image;

  image.stamp = ros_image.stamp.nanoseconds_since_epoch;
  image.name = ros_image.name;
  image.data = wrapImage(ros_image).clone();

  return image;
}

vision::ChannelImages copyImages(const vtr_messages::msg::ChannelImages &ros_channel) {
  vision::ChannelImages channel;
  channel.name = ros_channel.name;

  const auto& cameras = ros_channel.cameras;
  auto num_cameras = cameras.size();

  for(unsigned int idx = 0; idx < num_cameras; ++idx) {
    channel.cameras.emplace_back(copyImages(cameras[idx]));
  }

  return channel;
}

vision::RigImages copyImages(const vtr_messages::msg::RigImages &ros_rig) {
  vision::RigImages rig;
  rig.name = ros_rig.name;

  const auto& channels = ros_rig.channels;
  auto num_channels = channels.size();

  for(unsigned int idx = 0; idx < num_channels; ++idx) {
    auto channel = copyImages(channels[idx]);
    rig.channels.emplace_back(std::move(channel));
  }
  return rig;
}

vtr_messages::msg::Image copyImages(const vision::Image &asrl_image) {
  vtr_messages::msg::Image image;
  const auto &cv_image = asrl_image.data;
  image.stamp.nanoseconds_since_epoch = asrl_image.stamp;
  image.name = asrl_image.name;

  image.width = cv_image.cols;
  image.height = cv_image.rows;
  image.step = cv_image.step;
  if(cv_image.type() == CV_8UC1) {
    image.encoding = "mono8";
    image.depth = 1;
  } else if(cv_image.type() == CV_8UC3) {
    image.encoding = "bgr8";
    image.depth = 3;
  }

  auto datasize = image.step*image.height;
  image.data.resize(datasize);
  memcpy(&image.data[0],&cv_image.data[0],datasize);
  return image;
}

vtr_messages::msg::ChannelImages copyImages(const vision::ChannelImages &asrl_channel) {
  vtr_messages::msg::ChannelImages channel;
  channel.name = asrl_channel.name;
  for(auto &asrl_camera : asrl_channel.cameras) {
    channel.cameras.push_back(copyImages(asrl_camera));
  }

  return channel;
}

vtr_messages::msg::RigImages copyImages(const vision::RigImages &asrl_rig) {
  vtr_messages::msg::RigImages rig;
  rig.name = asrl_rig.name;
  for(auto &asrl_channel : asrl_rig.channels) {
    rig.channels.push_back(copyImages(asrl_channel));
  }
  return rig;
}

vision::Transform copyExtrinsics(const vtr_messages::msg::Transform  &ros_extrinsic) {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  Eigen::Vector3d axisangle = Eigen::Vector3d(ros_extrinsic.orientation.x,
                                              ros_extrinsic.orientation.y,
                                              ros_extrinsic.orientation.z);
  transform.block(0,0,3,3) = lgmath::so3::vec2rot(axisangle);
  transform(0,3) = ros_extrinsic.translation.x;
  transform(1,3) = ros_extrinsic.translation.y;
  transform(2,3) = ros_extrinsic.translation.z;
  return lgmath::se3::Transformation(transform);
}

vision::CameraIntrinsic copyIntrinsics(const vtr_messages::msg::CameraCalibration &ros_intrinsics) {
  vision::CameraIntrinsic intrinsic;
  for(int row = 0; row < 3; ++row) {
    for(int col = 0; col < 3; ++col) {
      intrinsic(row,col) = ros_intrinsics.k_mat[row*3+col];
    }
  }
  return intrinsic;
}

vision::RigCalibration copyCalibration(const vtr_messages::msg::RigCalibration &ros_calibration) {
  vision::RigCalibration calibration;

  calibration.rectified = ros_calibration.rectified;
  auto num_cameras = ros_calibration.intrinsics.size();
  for(unsigned int idx = 0; idx < num_cameras; ++idx) {
    calibration.intrinsics.push_back(copyIntrinsics(ros_calibration.intrinsics[idx]));
    calibration.extrinsics.push_back(copyExtrinsics(ros_calibration.extrinsics[idx]));
  }
  return calibration;
}

vision::RigCalibration copyCalibration(const vtr_messages::msg::XB3CalibrationResponse &ros_calibration) {
  vision::RigCalibration calibration;

  // the xb3 calibration is always rectified
  calibration.rectified = true;

  // fill out the extrinsics
  calibration.extrinsics.emplace_back();
  calibration.extrinsics.emplace_back();
  auto &right_extrinsics = calibration.extrinsics[1];
  Eigen::Matrix<double,4,4> right_extrinsic = Eigen::Matrix<double,4,4>::Identity();
  right_extrinsic(0,3) = -ros_calibration.baseline;
  right_extrinsics = lgmath::se3::Transformation(right_extrinsic);

  // fill out intrinsics
  Eigen::Matrix<double,3,3> intrinsic_matrix = Eigen::Matrix<double,3,3>::Identity();
  intrinsic_matrix(0,0) = ros_calibration.focal_length;
  intrinsic_matrix(0,2) = ros_calibration.optical_center_col;
  intrinsic_matrix(1,1) = ros_calibration.focal_length;
  intrinsic_matrix(1,2) = ros_calibration.optical_center_row;
  calibration.intrinsics.push_back(intrinsic_matrix);
  calibration.intrinsics.push_back(intrinsic_matrix);

  return calibration;
}

vtr_messages::msg::ChannelLandmarks copyLandmarks(const vision::ChannelLandmarks &asrl_landmarks) {
  vtr_messages::msg::ChannelLandmarks new_landmarks;
  new_landmarks.name = asrl_landmarks.name;

  auto lm_info = asrl_landmarks.appearance.feat_infos.cbegin();
  for(auto kp = asrl_landmarks.appearance.keypoints.cbegin(); kp != asrl_landmarks.appearance.keypoints.end(); ++kp, ++lm_info) {
    // copy over the feature info
    vtr_messages::msg::FeatureInfo ros_keypoint_info;
    ros_keypoint_info.laplacian_bit = lm_info->laplacian_bit;
    // precision isn't available in vtr_vision::vision_msgs::ChannelLandmarks
    ros_keypoint_info.scale = kp->octave;
    ros_keypoint_info.orientation = kp->angle;
    ros_keypoint_info.response = kp->response;

    new_landmarks.lm_info.push_back(ros_keypoint_info);
  }

#if 0
  for(const auto &vo_obs : asrl_landmarks.vo_obs) {
    // copy over the vo observations
  }
#endif

  for(int idx = 0; idx < asrl_landmarks.points.cols(); ++idx) {
    auto &point = asrl_landmarks.points.col(idx);
    vtr_messages::msg::HVec3 ros_point;
    ros_point.x = point(0);
    ros_point.y = point(1);
    ros_point.z = point(2);
    ros_point.w = 1.0;
    new_landmarks.points.push_back(ros_point);

    new_landmarks.num_vo_observations.push_back(1);
    auto &cov = asrl_landmarks.covariances.col(idx);
    for(int cov_idx = 0; cov_idx < 9; ++cov_idx) {
      new_landmarks.covariance.push_back(cov(cov_idx));
    }

    // update the validity
    new_landmarks.valid.push_back(asrl_landmarks.valid.at(idx));
  }

  // fill in the descriptor type
  new_landmarks.desc_type = copyDescriptorType(asrl_landmarks.appearance.feat_type);

  // memcpy the descriptors over.
  auto datasize = asrl_landmarks.appearance.descriptors.rows *
                  asrl_landmarks.appearance.feat_type.bytes_per_desc;
  new_landmarks.descriptors.resize(datasize);
  memcpy(&new_landmarks.descriptors[0], asrl_landmarks.appearance.descriptors.data, datasize);
  return new_landmarks;
}

#if 0
void updateLandmarks(vision_msgs::ChannelLandmarks &landmarks, const vision::ChannelLandmarks &asrl_landmarks) {

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
  return;
}
#endif

vtr_messages::msg::RigLandmarks copyLandmarks(const vision::RigLandmarks &asrl_landmarks) {
  vtr_messages::msg::RigLandmarks landmarks;
  landmarks.name = asrl_landmarks.name;
  for(const auto & asrl_channel : asrl_landmarks.channels) {
    landmarks.channels.push_back(copyLandmarks(asrl_channel));
  }
  return landmarks;
}

#if 0
void updateLandmarks(vision_msgs::RigLandmarks &landmarks, const vision::RigLandmarks &asrl_landmarks) {
  unsigned i = 0;
  for(const auto & asrl_channel : asrl_landmarks.channels) {
    auto *channel = landmarks.mutable_channels(i);
    updateLandmarks(*channel,asrl_channel);
    i++;
  }
  return;
}
#endif

vision::PersistentId copyPersistentId(const vtr_messages::msg::GraphPersistentId & persistent_id) {
  vision::PersistentId id;
  id.robot = persistent_id.robot;
  id.stamp = persistent_id.stamp;
  return id;
}

vtr_messages::msg::GraphPersistentId copyPersistentId(const vision::PersistentId & id) {
  vtr_messages::msg::GraphPersistentId persistent_id;
  persistent_id.robot = id.robot;
  persistent_id.stamp = id.stamp;
  return persistent_id;
}

vision::LandmarkId copyLandmarkId(const vtr_messages::msg::FeatureId &ros_id) {
  vision::LandmarkId id;

  id.index = ros_id.idx;
  id.camera = ros_id.camera;
  id.channel = ros_id.channel;
  id.rig = ros_id.rig;
  id.persistent = copyPersistentId(ros_id.persistent);
  return id;
}

vtr_messages::msg::FeatureId copyLandmarkId(const vision::LandmarkId &id) {
  vtr_messages::msg::FeatureId ros_id;
  ros_id.idx = id.index;
  ros_id.camera = id.camera;
  ros_id.channel = id.channel;
  ros_id.rig = id.rig;
  ros_id.persistent = copyPersistentId(id.persistent);
  return ros_id;
}

vision::Observations copyObservation(const vtr_messages::msg::Observations &ros_observation) {
  vision::Observations observations;
  observations.name = ros_observation.name;
  for(unsigned int kp_idx = 0; kp_idx < ros_observation.keypoints.size(); ++kp_idx) {

    // insert the 2D position
    const auto &ros_kp = ros_observation.keypoints[kp_idx];
    observations.points.emplace_back(vision::Point(ros_kp.position.x,
                                                   ros_kp.position.y));

    // insert the precision
    const auto &ros_precision = ros_observation.precisions[kp_idx];
    observations.precisions.emplace_back(ros_precision);
    // insert the covariances
    observations.covariances.emplace_back(Eigen::Matrix2d());
    auto &cov = observations.covariances.back();
    cov(0,0) = ros_observation.covariances[kp_idx*4];
    cov(0,1) = ros_observation.covariances[kp_idx*4+1];
    cov(1,0) = ros_observation.covariances[kp_idx*4+2];
    cov(1,1) = ros_observation.covariances[kp_idx*4+3];
  }

  for(const auto & ros_landmark : ros_observation.landmarks) {
    observations.landmarks.emplace_back(vision::LandmarkMatch());
    auto &landmark = observations.landmarks.back();
    landmark.from = copyLandmarkId(ros_landmark.from_id);
    for(const auto & obs_idx : ros_landmark.to_id) {
      landmark.to.push_back(copyLandmarkId(obs_idx));
    }
  }
  return observations;
}

vision::ChannelObservations copyObservation(const vtr_messages::msg::ChannelObservations &ros_observation) {
  vision::ChannelObservations observations;
  observations.name = ros_observation.name;
  for(const auto & camera : ros_observation.cameras) {
    observations.cameras.emplace_back(copyObservation(camera));
  }
  return observations;
}

vision::RigObservations copyObservation(const vtr_messages::msg::RigObservations &ros_observation) {
  vision::RigObservations observations;
  observations.name = ros_observation.name;
  for(const auto & channel : ros_observation.channels) {
    observations.channels.emplace_back(copyObservation(channel));
  }
  return observations;
}

vision::ChannelBowVocabulary copyChannelBowVocabulary(const vtr_messages::msg::ChannelBowVocabulary & ros_channel) {
  vision::ChannelBowVocabulary channel;
  channel.reserve(ros_channel.words.size());
  for (const auto & cluster : ros_channel.words) {
    channel.emplace_back(copyLandmarkId(cluster));
  }
  return channel;
}

vtr_messages::msg::ChannelBowVocabulary copyChannelBowVocabulary(const vision::ChannelBowVocabulary & channel) {
  vtr_messages::msg::ChannelBowVocabulary ros_vocab;
  for (const auto & word : channel) {
    ros_vocab.words.push_back(copyLandmarkId(word));
  }
  return ros_vocab;
}

vtr_messages::msg::RigBowVocabulary copyRigBowVocabulary(const vision::RigBowVocabulary & rig) {
  vtr_messages::msg::RigBowVocabulary ros_rig;
  for (const auto & channel : rig) {
    ros_rig.channels.push_back(copyChannelBowVocabulary(channel));
  }
  return ros_rig;
}

vision::RigBowVocabulary copyRigBowVocabulary(const vtr_messages::msg::RigBowVocabulary & ros_rig) {
  vision::RigBowVocabulary rig;
  rig.reserve(ros_rig.channels.size());
  for (const auto & channel : ros_rig.channels) {
    rig.emplace_back(copyChannelBowVocabulary(channel));
  }
  return rig;
}

vision::BowWordCount copyBowWordCount(const vtr_messages::msg::BowWordCount & ros_word_count) {
  vision::BowWordCount word_count;
  word_count.first = copyLandmarkId(ros_word_count.feature);
  word_count.second = ros_word_count.count;
  return word_count;
}

vtr_messages::msg::BowWordCount copyBowWordCount(const vision::BowWordCount & word_count) {
  vtr_messages::msg::BowWordCount ros_word_count;
  ros_word_count.feature = copyLandmarkId(word_count.first);
  ros_word_count.count = word_count.second;
  return ros_word_count;
}

vision::BowDescriptor copyBowDescriptor(const vtr_messages::msg::BowDescriptor & ros_bow) {
  vision::BowDescriptor bow;
  for (const auto & word_count : ros_bow.word_counts) {
    bow.insert(bow.end(), copyBowWordCount(word_count)); //< .end() is optimal if list is sorted
  }
  return bow;
}

vtr_messages::msg::BowDescriptor copyBowDescriptor(const vision::BowDescriptor & bow) {
  vtr_messages::msg::BowDescriptor ros_bow;
  for (const auto & word_count : bow) {
    ros_bow.word_counts.push_back(copyBowWordCount(word_count));
  }
  return ros_bow;
}

} // namespace messages
} // namespace vtr_vision
