#include <cmath>

#include <vtr_vision/features/extractor/orb_configuration.hpp>
#include <vtr_vision/features/extractor/orb_feature_extractor.hpp>

#include <vtr_logging/logging.hpp>

namespace vtr {
namespace vision {

using OFE = OrbFeatureExtractor;

void OFE::initialize(ORBConfiguration &config) {
  config_ = config;

  detector_ = cv::ORB::create(
      config_.num_detector_features_, config_.scaleFactor_, config_.nlevels_,
      config_.edgeThreshold_, config_.firstLevel_, config_.WTA_K_,
      static_cast<cv::ORB::ScoreType>(config_.scoreType_), config_.patchSize_,
      config_.fastThreshold_);

  stardetector_ = cv::xfeatures2d::StarDetector::create(
      config_.STAR_maxSize_, config_.STAR_responseThreshold_,
      config_.STAR_lineThresholdProjected_,
      config_.STAR_lineThresholdBinarized_, config_.STAR_suppressNonmaxSize_);

#if defined(HAVE_OPENCV_CUDAFEATURES2D)
  cudadetector_ = cv::cuda::ORB::create(
      config_.num_detector_features_, config_.scaleFactor_, config_.nlevels_,
      config_.edgeThreshold_, config_.firstLevel_, config_.WTA_K_,
      config_.scoreType_, config_.patchSize_, config_.fastThreshold_, true);
#endif  // defined(HAVE_OPENCV_CUDAFEATURES2D)
}

/////////////////////////////////////////////////////////////////////////
/// @brief Detect features on an image pyramid using STAR
void OFE::detectOnPyramid(const uMat &image, Keypoints &keypoints,
                          const cv::Mat &mask) {
  int &maxLevel = config_.nlevels_;

  // set up a temporary keypoints storage
  keypoints.reserve(config_.num_detector_features_);

  uMat src = image;
  cv::Mat src_mask = mask;
  cv::Mat dilated_mask;

  if (!mask.empty()) {
    dilate(mask, dilated_mask, cv::Mat());
    cv::Mat mask255(mask.size(), CV_8UC1, cv::Scalar(0));
    mask255.setTo(cv::Scalar(255), dilated_mask != 0);
    dilated_mask = mask255;
  }
  // for each pyramid level...
  for (int l = 0, multiplier = 1; l <= maxLevel; ++l, multiplier *= 2) {
    std::vector<cv::KeyPoint> new_pts;
    // detect on current level of the pyramid
    stardetector_->detect(src, new_pts, src_mask);

    // filter by the patch size
    cv::KeyPointsFilter::runByImageBorder(new_pts, src.size(),
                                          config_.patchSize_ / 2);

    // compute the orientation of the keypoint for descriptor generation
    if (!config_.upright_) {
      computeAngles(src, new_pts);
    }
    // ensure the keypoint is placed in the correct position at the bottom level
    // of the pyramid
    std::vector<cv::KeyPoint>::iterator it = new_pts.begin(),
                                        end = new_pts.end();
    for (; it != end; ++it) {
      it->pt.x *= multiplier;
      it->pt.y *= multiplier;
      it->size *= multiplier;
      it->octave = l;
    }

    // append to the list of keypoints
    keypoints.insert(keypoints.end(), new_pts.begin(), new_pts.end());

    // downsample the image using the pyramid
    if (l < maxLevel) {
      uMat dst;
      cv::pyrDown(src, dst);
      src = dst;

      if (!mask.empty()) {
        cv::resize(dilated_mask, src_mask, src.size(), 0, 0,
                   cv::INTER_AREA);  // vtr3 change: opencv 4+
      }
    }
  }
  if (!mask.empty()) {
    cv::KeyPointsFilter::runByPixelsMask(keypoints, mask);
  }

  // bin the keypoints
  binKeypoints(image.size(), keypoints);
}

void OFE::computeAngles(const uMat &image, Keypoints &keypoints) {
  cv::Mat accessor = image.getMat(cv::ACCESS_READ);

  const int halfPatchSize = 9;  // maximum allowable size by STAR detector
  size_t ptidx, ptsize = keypoints.size();
  // for each keypoint, sum up the moments
#pragma omp parallel for num_threads(config_.num_threads_)
  for (ptidx = 0; ptidx < ptsize; ptidx++) {
    int m_01 = 0, m_10 = 0;
    for (int u = -halfPatchSize; u <= halfPatchSize; ++u) {
      for (int v = -halfPatchSize; v <= halfPatchSize; ++v) {
        m_10 += u * accessor.at<uchar>(keypoints[ptidx].pt.y + v,
                                       keypoints[ptidx].pt.x + u);
        m_01 += v * accessor.at<uchar>(keypoints[ptidx].pt.y + v,
                                       keypoints[ptidx].pt.x + u);
      }
    }
    // calculate the orientation from the moments
    keypoints[ptidx].angle = cv::fastAtan2((float)m_01, (float)m_10);
  }
}

/////////////////////////////////////////////////////////////////////////
/// @brief Detect features on an image using the default ORB Harris/FAST
/// detector
void OFE::detectWithORB(const uMat &image, Keypoints &keypoints,
                        const cv::Mat &mask) {
  // reserve keypoints
  keypoints.reserve(config_.num_detector_features_);

  // detect keypoints (don't generate descriptors yet).
  detector_->detect(image, keypoints, cv::noArray());

  // bin the keypoints
  binKeypoints(image.size(), keypoints);
}

void OFE::binKeypoints(const cv::Size& size, Keypoints &keypoints) {
  // sanity check
  config_.x_bins_ = std::max(1, config_.x_bins_);
  config_.y_bins_ = std::max(1, config_.y_bins_);

  // figure out the number of buckets we need
  unsigned num_buckets = config_.x_bins_ * config_.y_bins_;

  // slightly inflate the number per bucket that we need
  unsigned desired_num_per_bucket =
      1.2 * config_.num_binned_features_ / num_buckets;

  // determine how many pixels there are in each bin
  float pixels_per_bin_x = float(size.width) / config_.x_bins_;
  float pixels_per_bin_y = float(size.height) / config_.y_bins_;

  // keep a record of the keypoints we bin
  std::map<std::pair<unsigned, unsigned>, std::vector<cv::KeyPoint *> >
      binned_keypoints;

  // make a new keypoints container
  Keypoints keypoints_new;
  keypoints_new.reserve(keypoints.size());

  for (auto & keypoint : keypoints) {
    unsigned bx = std::floor(keypoint.pt.x / pixels_per_bin_x);
    unsigned by = std::floor(keypoint.pt.y / pixels_per_bin_y);
    std::pair<unsigned, unsigned> pair(bx, by);
    binned_keypoints[pair].reserve(config_.num_detector_features_ /
                                   num_buckets);
    binned_keypoints[pair].push_back(&keypoint);
  }

  for (int ii = 0; ii < config_.x_bins_; ii++) {
    for (int jj = 0; jj < config_.y_bins_; jj++) {
      std::pair<unsigned, unsigned> pair(ii, jj);
      if (binned_keypoints[pair].size() > desired_num_per_bucket) {
        std::sort(binned_keypoints[pair].begin(), binned_keypoints[pair].end(),
                  [&](const cv::KeyPoint *k1, const cv::KeyPoint *k2) {
                    return k1->response > k2->response;
                  });
        binned_keypoints[pair].resize(desired_num_per_bucket);
      }
      for (unsigned kk = 0; kk < binned_keypoints[pair].size(); kk++) {
        // force keypoints to be upright if the config calls for it
        if (config_.upright_) {
          binned_keypoints[pair][kk]->angle = -1;
        }
        keypoints_new.push_back(*binned_keypoints[pair][kk]);
      }
    }
  }

  // now place the upper cap on the number of features
  std::sort(keypoints_new.begin(), keypoints_new.end(),
            [&](const cv::KeyPoint &k1, const cv::KeyPoint &k2) {
              return k1.response > k2.response;
            });
  if ((int)keypoints_new.size() > config_.num_binned_features_) {
    keypoints_new.resize(config_.num_binned_features_);
  }

  // clear out and copy new keypoints
  keypoints.clear();
  keypoints = keypoints_new;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Extracts a list of descriptors and keypoints from a single image.
Features OFE::extractFeatures(const cv::Mat &image) {
  uMat uimage;
  image.copyTo(uimage);
  // create a new empty frame
  Features frame;
  frame.keypoints.reserve(config_.num_detector_features_);
  frame.feat_infos.reserve(config_.num_detector_features_);
  frame.feat_type.impl = FeatureImpl::OPENCV_ORB;
  frame.feat_type.upright = config_.upright_;

  // make an empty mask. the feature detectors need this but we don't use it
  cv::Mat mask;

  // do the detection
  if (config_.use_STAR_detector_) {
    // use the STAR detector on a pyramid
    detectOnPyramid(uimage, frame.keypoints, mask);
  } else {
    // use ORB's default HARRIS or FAST detectors
    detectWithORB(uimage, frame.keypoints, mask);
  }

  // create a new feature detector
  if (config_.use_GPU_descriptors_) {
#if defined(HAVE_OPENCV_CUDAFEATURES2D)
    // we have to sort by the octave or the GPU descriptors get messed up for
    // some reason
    std::sort(frame.keypoints.begin(), frame.keypoints.end(),
              [&](const cv::KeyPoint &k1, const cv::KeyPoint &k2) {
                return k1.octave < k2.octave;
              });
    // the GPU can only compute descriptors one at a time, so need a mutex here
    {
      std::lock_guard<std::mutex> lock(__gpu_mutex__);
      cv::cuda::GpuMat gpuimage(image);
      cudadetector_->compute(gpuimage, frame.keypoints, frame.gpu_descriptors);
      frame.gpu_descriptors.download(frame.descriptors);
    }
#else   // defined(HAVE_OPENCV_CUDAFEATURES2D)
    LOG(ERROR)
        << "You asked vtr_vision::OrbFeatureMatcher::extractFeatures() to "
           "use the GPU for descriptor generation, but OpenCV wasn't built "
           "with this support configured!";
#endif  // defined(HAVE_OPENCV_CUDAFEATURES2D)
  } else {
    detector_->compute(uimage, frame.keypoints, frame.descriptors);
  }

  // fill out the feature infos (this is after descriptor computation,
  // as some keypoints may be removed by the descriptor computation)
  for (unsigned kk = 0; kk < frame.keypoints.size(); kk++) {
    double sigma =
        config_.use_STAR_detector_
            ? std::pow(2, frame.keypoints[kk].octave)
            : std::pow(config_.scaleFactor_, frame.keypoints[kk].octave);
    double sigma_squared = sigma * sigma;
    bool laplacian_bit = static_cast<int>(frame.keypoints[kk].response) & 0x1;
    frame.feat_infos.emplace_back();
    FeatureInfo &info = frame.feat_infos.back();
    info.laplacian_bit = laplacian_bit;
    info.precision = 1.0 / sigma_squared;
    info.covariance(0, 0) = sigma_squared;
    info.covariance(1, 1) = sigma_squared;
    info.covariance(0, 1) = 0.0;
    info.covariance(1, 0) = 0.0;
  }

  frame.feat_type.dims = frame.descriptors.step1();
  frame.feat_type.bytes_per_desc = frame.descriptors.step1();
  return frame;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Extracts a list of descriptors and keypoints from a set of two
/// rectified stereo images.
ChannelFeatures OFE::extractStereoFeatures(
    const cv::Mat &left_img, const cv::Mat &right_img) {
  // create a new empty frame
  ChannelFeatures features_temp;
  features_temp.cameras.reserve(2);

  // make two asynchronous calls to extract the features
  Features (BaseFeatureExtractor::*extract_func)(
      const cv::Mat &) = &BaseFeatureExtractor::extractFeatures;
  auto handle_left =
      std::async(std::launch::async, extract_func, this, left_img);
  auto handle_right =
      std::async(std::launch::async, extract_func, this, right_img);
  features_temp.cameras.push_back(handle_left.get());
  features_temp.cameras.push_back(handle_right.get());

  // We need to match for the stereo case
  vtr::vision::ASRLFeatureMatcher::Config matcher_config =
      config_.stereo_matcher_config_;
  matcher_config.descriptor_match_thresh_ =
      config_.stereo_matcher_config_.stereo_descriptor_match_thresh_;
  // set the number of threads
  matcher_config.num_threads_ = 8;
  vtr::vision::ASRLFeatureMatcher matcher(matcher_config);

  // create the new empty channel features, copy descriptor type info
  ChannelFeatures features;
  features.cameras.resize(2);
  auto cam_rng = {0, 1};
  for (auto &i : cam_rng)
    features.cameras[i].feat_type = features_temp.cameras[i].feat_type;

  // perform matching
  SimpleMatches matches;
  if (calib_.rectified || calib_.extrinsics.size() < 2) {
    // if the rig is rectified or not set, just do regular stereo matching
    matches = matcher.matchStereoFeatures(features_temp.cameras[0],
                                          features_temp.cameras[1]);
  } else {
    // if not rectified, we need to do epipolar matching
    auto tf = calib_.extrinsics[0].inverse() * calib_.extrinsics[1];

    // pre-cache some data to make the operations easier to read
    CameraIntrinsic &K0 = calib_.intrinsics[0];
    CameraIntrinsic &K1 = calib_.intrinsics[1];
    Eigen::Matrix3d K1it = K1.transpose().inverse();
    Eigen::Vector3d KrtT = K0 * tf.C_ba().transpose() * tf.r_ba_ina();

    // turn KrtT into skew symmetric form
    Eigen::Matrix3d KrtTcross;
    KrtTcross << 0, -KrtT(2), KrtT(1), KrtT(2), 0, -KrtT(0), -KrtT(1), KrtT(0),
        0;

    // make the fundamental matrix
    Eigen::Matrix3d F = K1it * tf.C_ba() * K0.transpose() * KrtTcross;

    // do matching
    matches = matcher.matchFeatures(
        features_temp.cameras[0], features_temp.cameras[1], F,
        matcher_config.stereo_x_tolerance_min_,
        matcher_config.stereo_x_tolerance_max_,
        matcher_config.stereo_y_tolerance_,
        ASRLFeatureMatcher::CheckType::EPIPOLE);
  }

  // reserve the corresponding vectors to match
  const auto &desc_cols = features_temp.cameras[0].descriptors.cols;
  const auto &desc_cvtype = features_temp.cameras[0].descriptors.type();
  for (unsigned j : cam_rng) {
    Features &j_feat = features.cameras[j];
    j_feat.keypoints.reserve(matches.size());
    j_feat.feat_infos.reserve(matches.size());
    j_feat.descriptors = cv::Mat(matches.size(), desc_cols, desc_cvtype);
  }

  // our stereo frames have the special requirement that all
  // features much have a match, and the matches index-aligned
  for (unsigned i = 0; i < matches.size(); i++) {
    for (unsigned j : cam_rng) {
      const auto &temp_idx = j == 0 ? matches[i].first : matches[i].second;
      Features &j_feat = features.cameras[j];
      Features &j_temp_feat = features_temp.cameras[j];
      j_feat.keypoints.push_back(j_temp_feat.keypoints[temp_idx]);
      j_feat.feat_infos.push_back(j_temp_feat.feat_infos[temp_idx]);
      j_temp_feat.descriptors.row(temp_idx).copyTo(j_feat.descriptors.row(i));
    }
  }

  return features;
}

}  // namespace vision
}  // namespace vtr
