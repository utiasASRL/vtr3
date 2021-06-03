#include <algorithm>
#include <memory>
#include <numeric>
#include <vector>

#include <vtr_vision/features/extractor/cuda/gpu_surf_feature_extractor.hpp>

namespace vtr {
namespace vision {

using GSFE = GpuSurfFeatureExtractor;

std::mutex GSFE::gpu_mutex_;

bool checkVariances(float xx, float yy) {
  // filter out the features with saddle-points for variances
  if ((xx < 0 && yy > 0) || (xx > 0 && yy < 0)) {
    return false;
  }

  // filter out the features with super-enormous variances
  return !(xx > 10000 || yy > 10000);
}

void fixCovariance(Eigen::Matrix2d &cov) {
  if (cov(0, 0) < 0) {
    cov = cov * -1;
  }

  // Initialize an eigen value solver
  // TODO: (old) Some are not pos. definite.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 2, 2>> eigsolver(
      cov, Eigen::EigenvaluesOnly);
  // Check the minimum eigen value
  if (eigsolver.eigenvalues().minCoeff() <= 0) {
    cov(0, 1) = 0;
    cov(1, 0) = 0;
  }
}

Keypoint convert(const asrl::Keypoint &kp_in) {
  Keypoint kp_out;
  kp_out.pt.x = kp_in.x;
  kp_out.pt.y = kp_in.y;
  kp_out.angle = kp_in.angle;
  kp_out.octave = kp_in.octave;
  kp_out.response = kp_in.response;
  return kp_out;
}

////////////////////////////////////////////////////////////////////////////////
void GSFE::initialize(asrl::GpuSurfConfiguration &config) {
  config_ = config;
  detector_ = std::make_unique<asrl::GpuSurfDetector>(config_);
}

////////////////////////////////////////////////////////////////////////////////
void GSFE::initialize(asrl::GpuSurfStereoConfiguration &stereo_config) {
  stereo_config_ = stereo_config;
  stereo_detector_ =
      std::make_unique<asrl::GpuSurfStereoDetector>(stereo_config_);
}

////////////////////////////////////////////////////////////////////////////////
Features GSFE::SURFToFrame(const std::vector<asrl::Keypoint> &keypoints,
                           const std::vector<float> &descriptors) {
  // make a frame to return
  Features features;
  // we need to convert the vtr_vision keypoints to opencv keypoints
  features.keypoints.reserve(keypoints.size());
  features.feat_infos.reserve(keypoints.size());
  std::vector<bool> valid_keypoint;
  valid_keypoint.resize(keypoints.size(), true);
  for (unsigned i = 0; i < keypoints.size(); i++) {
    // check the covariance
    if (!checkVariances(keypoints[i].sigma_xx, keypoints[i].sigma_yy)) {
      valid_keypoint[i] = false;
      continue;
    }
    features.keypoints.emplace_back();
    auto &kpt = features.keypoints.back();
    kpt.pt.x = keypoints[i].x;
    kpt.pt.y = keypoints[i].y;
    kpt.angle = keypoints[i].angle;
    kpt.octave = keypoints[i].octave;
    kpt.response = keypoints[i].response;
    kpt.size = keypoints[i].size;
    // augment with data that isn't in the cv::KeyPoint struct
    double sigma = (1 << static_cast<int>(keypoints[i].octave));
    bool laplacian_bit = static_cast<int>(keypoints[i].response) & 0x1;
    auto precision = 1.0 / std::pow(sigma, 2.0);
    features.feat_infos.emplace_back(laplacian_bit, precision);
    auto &info = features.feat_infos.back();
    info.covariance(0, 0) = keypoints[i].sigma_xx;
    info.covariance(1, 1) = keypoints[i].sigma_yy;
    info.covariance(0, 1) = keypoints[i].sigma_xy;
    info.covariance(1, 0) = keypoints[i].sigma_xy;
    fixCovariance(info.covariance);
  }

  // get the descriptor size
  unsigned descriptor_size = detector_->descriptorSize();

  // set the upright flag
  features.feat_type.upright = config_.upright_flag;

  // set the feature type
  features.feat_type.impl = FeatureImpl::ASRL_GPU_SURF;

  // copy into the Frame's cv::Mat
  features.descriptors =
      cv::Mat_<float>(features.keypoints.size(), descriptor_size);

  auto *cv_data = (float *)features.descriptors.data;
  for (unsigned idx = 0; idx < valid_keypoint.size(); ++idx) {
    if (valid_keypoint[idx]) {
      memcpy(cv_data, &descriptors[idx * descriptor_size],
             sizeof(float) * descriptor_size);
      cv_data += 64;
    }
  }

  // set the dimensions
  features.feat_type.dims = descriptor_size;

  // set bytes per descriptor
  features.feat_type.bytes_per_desc = features.feat_type.dims * sizeof(float);

  return features;
}

////////////////////////////////////////////////////////////////////////////////
Features GSFE::extractFeatures(const cv::Mat &image) {
  // we're about to use the gpu, lock
  std::unique_lock<std::mutex> lock(gpu_mutex_);

  // get vtr_vision keypoints
  std::vector<asrl::Keypoint> keypoints;
  // detect and generate descriptors
  detector_->buildIntegralImage(image);
  detector_->detectKeypoints();
  if (!config_.upright_flag) {
    detector_->findOrientation();
  }
  detector_->computeDescriptors(false);

  // get keypoints
  detector_->getKeypoints(keypoints);

  // get descriptors
  std::vector<float> descriptors;
  detector_->getDescriptors(descriptors);

  // We're done with the gpu, unlock
  lock.unlock();

  // return the frame
  return SURFToFrame(keypoints, descriptors);
}

////////////////////////////////////////////////////////////////////////////////
ChannelFeatures GSFE::extractStereoFeatures(const cv::Mat &left_img,
                                            const cv::Mat &right_img) {
  // we're about to use the gpu, lock
  std::unique_lock<std::mutex> lock(gpu_mutex_);
  stereo_detector_->setImages(left_img, right_img);
  stereo_detector_->detectKeypoints();
  if (!stereo_config_.upright_flag) {
    stereo_detector_->findOrientation();
  }
  stereo_detector_->computeDescriptors(false);
  stereo_detector_->matchKeypoints();

  // get keypoints
  std::vector<std::vector<asrl::Keypoint>> keypoints;
  keypoints.resize(2);
  std::vector<int> leftRightMatches;
  stereo_detector_->getKeypoints(keypoints[0], keypoints[1], leftRightMatches);
  // get descriptors
  std::vector<float> descriptors;
  stereo_detector_->getDescriptors(descriptors);
  int descriptor_size = stereo_detector_->descriptorSize();
  // We're done with the gpu, unlock
  lock.unlock();

  std::vector<std::size_t> p(keypoints[0].size());
  std::iota(p.begin(), p.end(), 0);
#ifdef DETERMINISTIC_VTR
  // sort keypoints and descriptors in same way so deterministic
  auto sort_func = [](asrl::Keypoint const &a, asrl::Keypoint const &b) {
    if (a.x < b.x || (a.x == b.x && a.y < b.y))
      return true;
    else
      return false;
  };
  std::sort(p.begin(), p.end(), [&](std::size_t i, std::size_t j) {
    return sort_func(keypoints[0][i], keypoints[0][j]);
  });
#endif

  // count the number of good matches we have
  unsigned num_good_matches = 0;
  for (auto &m : leftRightMatches) num_good_matches += m != -1;

  // set up the stereo feature output
  ChannelFeatures channel;
  channel.cameras.resize(2);
  auto &left_feat = channel.cameras[0];
  auto &right_feat = channel.cameras[1];
  auto cam_rng = {0, 1};
  for (auto &i : cam_rng) {
    channel.cameras[i].keypoints.reserve(num_good_matches);
    channel.cameras[i].feat_infos.reserve(num_good_matches);
    channel.cameras[i].feat_type.upright = stereo_config_.upright_flag;
    channel.cameras[i].feat_type.impl = FeatureImpl::ASRL_GPU_SURF;
    channel.cameras[i].feat_type.dims = 64;
    channel.cameras[i].feat_type.bytes_per_desc =
        channel.cameras[i].feat_type.dims * sizeof(float);
  }
  // int feature_count = 0;
  std::vector<bool> valid_keypoint;
  valid_keypoint.resize(leftRightMatches.size(), true);
  for (unsigned i = 0; i < leftRightMatches.size(); i++) {
    if (leftRightMatches[p[i]] != -1) {
      const auto &left_asrl_feat = keypoints[0][p[i]];
      const auto &right_asrl_feat = keypoints[1][leftRightMatches[p[i]]];
      // check the covariance
      if (!checkVariances(left_asrl_feat.sigma_xx, left_asrl_feat.sigma_yy) ||
          !checkVariances(right_asrl_feat.sigma_xx, right_asrl_feat.sigma_yy)) {
        valid_keypoint[p[i]] = false;
        continue;
      }
      // set up the left keypoint
      left_feat.keypoints.emplace_back(convert(left_asrl_feat));

      // augment with data that isn't in the cv::KeyPoint struct
      double sigma = (1 << static_cast<int>(left_asrl_feat.octave));
      bool laplacian_bit = static_cast<int>(left_asrl_feat.response) & 0x1;
      left_feat.feat_infos.emplace_back(laplacian_bit,
                                        1.0 / std::pow(sigma, 2.0));

      auto &info = left_feat.feat_infos.back();
      info.covariance(0, 0) = left_asrl_feat.sigma_xx;
      info.covariance(1, 1) = left_asrl_feat.sigma_yy;
      info.covariance(0, 1) = left_asrl_feat.sigma_xy;
      info.covariance(1, 0) = left_asrl_feat.sigma_xy;
      fixCovariance(info.covariance);

      // set up the matching right keypoint
      right_feat.keypoints.emplace_back(convert(right_asrl_feat));

      // augment with data that isn't in the cv::KeyPoint struct
      sigma = (1 << static_cast<int>(right_asrl_feat.octave));
      laplacian_bit = static_cast<int>(right_asrl_feat.response) & 0x1;
      right_feat.feat_infos.emplace_back(laplacian_bit,
                                         1.0 / std::pow(sigma, 2.0));

      auto &right_info = right_feat.feat_infos.back();
      right_info.covariance(0, 0) = right_asrl_feat.sigma_xx;
      right_info.covariance(1, 1) = right_asrl_feat.sigma_yy;
      right_info.covariance(0, 1) = right_asrl_feat.sigma_xy;
      right_info.covariance(1, 0) = right_asrl_feat.sigma_xy;
      fixCovariance(right_info.covariance);
    }
  }

  // TODO: (old) Copy Descriptors
  left_feat.descriptors =
      cv::Mat(left_feat.keypoints.size(), descriptor_size, CV_32F);
  auto *cv_data = (float *)left_feat.descriptors.data;
  for (unsigned idx = 0; idx < leftRightMatches.size(); ++idx) {
    if (leftRightMatches[p[idx]] != -1 && valid_keypoint[p[idx]]) {
      memcpy(cv_data, &descriptors[p[idx] * descriptor_size],
             sizeof(float) * descriptor_size);
      cv_data += 64;
    }
  }

  return channel;
}

}  // namespace vision
}  // namespace vtr
