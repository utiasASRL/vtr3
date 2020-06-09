#pragma once

#include <random>

#include <vtr/navigation/modules/base_module.h>
#include <vtr/navigation/modules/ransac/ransac_module.h>

#include <asrl/messages/Matches.pb.h>
//#include <asrl/vision/matching/SensorModel/SensorModelBase.h>
#include <asrl/vision/outliers.hpp>

namespace asrl {
namespace navigation {

/** \brief Reject outliers and estimate a preliminary transform using Stereo
 * Data
 */
class StereoRansacModule : public RansacModule {
 public:
  static constexpr auto type_str_ = "stereo_ransac";
  struct Config : RansacModule::Config {
    double mask_depth;
    double mask_depth_inlier_count;
    // Whether or not to use landmark covariance for the error model
    bool use_covariance = false;
  };

  /** \brief TODO Construct with settings...
   */
  StereoRansacModule()
      : doom_twister(vo_doom_generator()), doom_distribution(0, 100) {}

  /** \brief Update the graph with the frame data for the live vertex
   */
  virtual void updateGraph(QueryCache &, MapCache &,
                           const std::shared_ptr<Graph> &, VertexId){};

  void setConfig(std::shared_ptr<Config> &config);

 protected:
  /** \brief Generates a model for the RANSAC method.
   *
   * \param[in] qdata, the reference frame. position of this frame is locked
   * and set to the origin.
   * \param[in] mdata, The frame whose position is being optimized.
   * \return a pointer to the RANSAC model.
   */
  virtual std::shared_ptr<vision::SensorModelBase<Eigen::Matrix4d>>
  generateRANSACModel(QueryCache &qdata, MapCache &mdata);

  /** \brief Generates a sampler for the RANSAC method.
   *
   * \param[in] qdata, the reference frame. position of this frame is locked
   * and set to the origin.
   * \param[in] mdata, The frame whose position is being optimized.
   * \return a pointer to the RANSAC model.
   */
  virtual std::shared_ptr<vision::BasicSampler> generateRANSACSampler(
      QueryCache &qdata, MapCache &mdata);

 private:
  /** \brief Adds points to the ransac problem given a set of landmarks.
   *
   * \param[in,out] ransac_points the points to be added to
   * \param[in] landmarks the landmarks.
   * \param[in] a set of offsets corresponding to each channel.
   */
  void addPointsFromLandmarks(
      Eigen::Matrix<double, 3, Eigen::Dynamic> &ransac_points,
      const vision::RigLandmarks &landmarks, OffsetMap &channel_offsets);

  /** \brief Adds covariances to the ransac problem given a set of landmarks.
   *
   * \param[in,out] inv_r_matrix the covariant to be added to
   * \param[in] landmarks the landmarks.
   * \param[in] a set of offsets corresponding to each channel.
   */
  void setCovarianceFromObservations(vision::MeasVarList &inv_r_matrix,
                                     const vision::RigObservations &landmarks,
                                     OffsetMap &);

  /** \brief The flattened query points.
   */
  std::shared_ptr<EigenMatrix3Dynamic> query_points_;

  /** \brief The flattened map points.
   */
  std::shared_ptr<EigenMatrix3Dynamic> map_points_;

  std::shared_ptr<Config> stereo_config_;

  std::random_device vo_doom_generator;  // non-deterministic generator
  std::mt19937 doom_twister;  // (rd());  // to seed mersenne twister.
                              // replace the call to rd() with a
                              // constant value to get repeatable
                              // results.
  std::uniform_real_distribution<double> doom_distribution;
};

}  // namespace navigation
}  // namespace asrl
