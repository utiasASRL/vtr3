#pragma once

#include <random>

#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_tactic/modules/stereo/ransac/ransac_module.hpp>
#include <vtr_vision/outliers.hpp>
#include <vtr_vision/outliers/sampler/basic_sampler.hpp>
#include <vtr_vision/outliers/sampler/verify_sample_subset_mask.hpp>
#include <vtr_vision/sensors/stereo_transform_model.hpp>

using EigenMatrix3Dynamic = Eigen::Matrix<double, 3, Eigen::Dynamic>;

namespace vtr {
namespace tactic {

/**
 * \brief Reject outliers and estimate a preliminary transform using Stereo
 * Data
 * \details
 * requires:
 *   qdata.[candidate_landmarks, rig_features, rig_calibrations,
 *          T_sensor_vehicle, live_id],
 *   mdata.[map_landmarks, T_q_m_prior, T_sensor_vehicle_map, raw_matches]
 * outputs:
 *   mdata.[T_q_m, ransac_matches, success, **steam_failure]
 *
 * The preliminary transform is stored in T_q_m, and inliers stored in
 * ransac_matches.
 * \todo (yuchen) why is steam_failure relevant in this module? likely for
 * testing only, should be able to remove it
 */
class StereoRansacModule : public RansacModule {
 public:
  /**
   * \brief Static module identifier.
   * \todo change this to static_name
   */
  static constexpr auto static_name = "stereo_ransac";

  /** \brief Collection of config parameters */
  struct Config : RansacModule::Config {
    double mask_depth;
    int mask_depth_inlier_count;
    // Whether or not to use landmark covariance for the error model
    bool use_covariance = false;
  };

  /** \brief TODO Construct with settings... */
  StereoRansacModule(std::string name = static_name)
      : RansacModule{name},
        doom_twister(vo_doom_generator()),
        doom_distribution(0, 100){};

  void setConfig(std::shared_ptr<Config> &config);

 protected:
  /** \brief Generates a model for the RANSAC method.
   *
   * \param[in] qdata The reference frame. position of this frame is locked
   * and set to the origin.
   * \param[in] mdata The frame whose position is being optimized.
   * \return A pointer to the RANSAC model.
   */
  virtual std::shared_ptr<vision::SensorModelBase<Eigen::Matrix4d>>
  generateRANSACModel(QueryCache &qdata, MapCache &mdata);

  /** \brief Generates a sampler for the RANSAC method.
   *
   * \param[in] qdata The reference frame. position of this frame is locked
   * and set to the origin.
   * \param[in] mdata The frame whose position is being optimized.
   * \return A pointer to the RANSAC model.
   */
  virtual std::shared_ptr<vision::BasicSampler> generateRANSACSampler(
      QueryCache &qdata, MapCache &mdata);

 private:
  /**
   * \brief Adds points to the ransac problem given a set of landmarks.
   * \param[in,out] ransac_points the points to be added to
   * \param[in] landmarks the landmarks.
   * \param[in] channel_offsets a set of offsets corresponding to each channel.
   */
  void addPointsFromLandmarks(
      Eigen::Matrix<double, 3, Eigen::Dynamic> &ransac_points,
      const vision::RigLandmarks &landmarks, OffsetMap &channel_offsets);

  /**
   * \brief Adds covariances to the ransac problem given a set of landmarks.
   * \param[in,out] inv_r_matrix the covariant to be added to
   * \param[in] landmarks the landmarks.
   */
  void setCovarianceFromObservations(vision::MeasVarList &inv_r_matrix,
                                     const vision::RigObservations &landmarks,
                                     OffsetMap &);

  /** \brief The flattened query points. */
  std::shared_ptr<EigenMatrix3Dynamic> query_points_;

  /** \brief The flattened map points. */
  std::shared_ptr<EigenMatrix3Dynamic> map_points_;

  std::shared_ptr<Config> stereo_config_;

  std::random_device vo_doom_generator;  // non-deterministic generator
  std::mt19937 doom_twister;  // (rd());  // to seed mersenne twister.
                              // replace the call to rd() with a
                              // constant value to get repeatable
                              // results.
  std::uniform_real_distribution<double> doom_distribution;
};

}  // namespace tactic
}  // namespace vtr
