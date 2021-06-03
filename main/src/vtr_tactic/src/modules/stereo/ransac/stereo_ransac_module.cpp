#include <vtr_tactic/modules/stereo/ransac/stereo_ransac_module.hpp>

namespace vtr {
namespace tactic {

void StereoRansacModule::setConfig(std::shared_ptr<Config> &config) {
  // Set the base module
  auto down_casted_config =
      std::dynamic_pointer_cast<RansacModule::Config>(config);
  RansacModule::setConfig(down_casted_config);
  stereo_config_ = config;
}

std::shared_ptr<vision::SensorModelBase<Eigen::Matrix4d>>
StereoRansacModule::generateRANSACModel(QueryCache &qdata, MapCache &) {
  // Add this back in if you want to test your robustness to VO failures.
  //  auto vo_doom = doom_distribution(doom_twister);
  //  if(vo_doom >= 99.5 && stereo_config_->use_migrated_points == false) {
  //    LOG(ERROR) << "DOOM TO YOUR VO " << THE_DEVIL;
  //    return nullptr;
  //  }
  // Get the data from the cache.
  const auto &calibrations = *qdata.rig_calibrations;

  // set up the model.
  auto stereo_model = std::make_shared<vision::StereoTransformModel>();

  // set up the measurement variance
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> inv_r_matrix;

  // Set up the map and query points based on whether
  // we are configured to use migrated points or not.
  if (stereo_config_->use_migrated_points) {
    auto &query_landmarks = *qdata.map_landmarks;
    auto map_points = &(*qdata.migrated_points_3d);

    // clear any old data
    query_points_.reset(new EigenMatrix3Dynamic);

    // grab the points from the landmarks and put them in a contiguous matrix
    addPointsFromLandmarks(*query_points_.get(), query_landmarks[0].landmarks,
                           query_channel_offsets_);

    // add the map and query points to the stereo model
    stereo_model->setPoints(map_points, query_points_.get());

    // keep a record of the channel offsets
    for (uint32_t channel_idx = 0; channel_idx < query_channel_offsets_.size();
         ++channel_idx) {
      map_channel_offsets_[channel_idx] =
          std::pair<int, int>(0, map_points->cols());
    }
    // TODO: (old) Add migrated covariance in, need to set up noise evaluator
    // based on prior in landmark migration module.

  } else {
    auto &query_landmarks = *qdata.candidate_landmarks;
    auto &map_landmarks = *qdata.map_landmarks;

    // clear any old data
    map_points_.reset(new EigenMatrix3Dynamic);
    query_points_.reset(new EigenMatrix3Dynamic);

    // grab the points from the landmarks and put them in a contiguous matrix
    addPointsFromLandmarks(*map_points_.get(), map_landmarks[0].landmarks,
                           map_channel_offsets_);
    addPointsFromLandmarks(*query_points_.get(), query_landmarks[0],
                           query_channel_offsets_);

    // set the covariances for each observation
    if (stereo_config_->use_covariance) {
      setCovarianceFromObservations(inv_r_matrix, map_landmarks[0].observations,
                                    map_channel_offsets_);
    }

    // add the measurement variances to the stereo model
    stereo_model->setMeasurementVariance(inv_r_matrix);

    // add the map and query points to the stereo model
    stereo_model->setPoints(map_points_.get(), query_points_.get());
  }

  // TODO: (old) Use covariance in RANSAC for migrated points.
  // TODO: (old) Multi-Channel.

  // Set up the calibration.
  auto extrinsic = calibrations.front().extrinsics[1];
  auto baseline = -extrinsic.matrix()(0, 3);
  auto intrinsic = calibrations.front().intrinsics[0];
  stereo_model->setCalibration(intrinsic, baseline);

  return stereo_model;
}

void StereoRansacModule::setCovarianceFromObservations(
    vision::MeasVarList &inv_r_matrix,
    const vision::RigObservations &observations, OffsetMap &) {
  // figure out how many landmarks there are across all of the channels.
  int num_landmarks = 0;
  for (auto &channel : observations.channels) {
    if (!channel.cameras.empty())
      num_landmarks += channel.cameras[0].covariances.size();
  }

  // set up the points for the ransac problem.
  int lm_idx = 0;
  inv_r_matrix.conservativeResize(2, 2 * (num_landmarks + inv_r_matrix.cols()));

  // Go through every channel.
  for (const auto &channel : observations.channels) {
    // keep track of the offset into this channel.
    if (!channel.cameras.empty()) {
      for (unsigned idx = 0; idx < channel.cameras[0].precisions.size();
           idx++) {
        // grab the inverse covariances
        inv_r_matrix.block(0, 2 * (lm_idx + idx), 2, 2) =
            channel.cameras[0].covariances[idx].inverse();
      }
      lm_idx += channel.cameras[0].precisions.size();
    }
  }
}

void StereoRansacModule::addPointsFromLandmarks(
    Eigen::Matrix<double, 3, Eigen::Dynamic> &ransac_points,
    const vision::RigLandmarks &landmarks, OffsetMap &channel_offsets) {
  // figure out how many landmarks there are across all of the channels.
  int num_landmarks = 0;
  for (auto &channel : landmarks.channels) {
    num_landmarks += channel.points.cols();
  }

  // set up the points for the ransac problem.
  int lm_idx = 0;
  ransac_points.conservativeResize(3, num_landmarks + ransac_points.cols());

  // Go through every channel.
  for (uint32_t channel_idx = 0; channel_idx < landmarks.channels.size();
       channel_idx++) {
    auto &channel = landmarks.channels[channel_idx];
    // keep track of the offset into this channel.
    channel_offsets[channel_idx] =
        std::pair<int, int>(lm_idx, lm_idx + channel.points.cols());
    if (channel.points.cols() > 0) {
      // set the point.
      ransac_points.block(0, lm_idx, 3, channel.points.cols()) = channel.points;
      lm_idx += channel.points.cols();
    }
  }
}

std::shared_ptr<vision::BasicSampler> StereoRansacModule::generateRANSACSampler(
    QueryCache &qdata, MapCache &) {
  auto &query_landmarks = *qdata.candidate_landmarks;

  std::vector<bool> mask;
  for (auto &rig : query_landmarks) {
    for (auto &channel : rig.channels) {
      for (uint32_t lm_idx = 0; lm_idx < channel.points.cols(); ++lm_idx) {
        mask.push_back(channel.points(2, lm_idx) < stereo_config_->mask_depth);
      }
    }
  }

  auto verifier = std::make_shared<vision::VerifySampleSubsetMask>(
      stereo_config_->mask_depth_inlier_count, mask);  // Need 1 close feature
  return std::make_shared<vision::BasicSampler>(verifier);
}

}  // namespace tactic
}  // namespace vtr
