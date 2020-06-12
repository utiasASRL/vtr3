#pragma once

#include <vtr/navigation/modules/base_module.h>

#include <asrl/messages/Matches.pb.h>
#include <asrl/vision/outliers/sampler/BasicSampler.hpp>
#include <asrl/vision/sensors/SensorModelBase.hpp>

namespace vtr {
namespace navigation {

/** \brief The base RANSAC module.
 */
class RansacModule : public BaseModule {
 public:
  using OffsetMap = std::map<uint32_t, std::pair<uint32_t, uint32_t>>;

  /** \brief Static module identifier.
   *
   * \todo change this to static_name
   */
  static constexpr auto type_str_ = "ransac";

  /** \brief Collection of config parameters
   */
  struct Config {
    /** \brief Ransac enable flag. If disabled, the ransac module simply serves
     * as a passthrough
     */
    bool enable;

    /** \brief Max iterations for RANSAC.
     */
    int iterations;

    /** \brief Type of RANSAC to use. Options [ Vanilla ].
     */
    std::string flavor;

    /** \brief Std. deviation of the keypoint uncertainty, in pixels.
     */
    double sigma;

    /** \brief Threshold on inlier, in std. deviations.
     */
    double threshold;

    /** \brief Minimum ratio of inliers to outliers needed for early stop.
     */
    double early_stop_ratio;

    /** \brief Minimum number of inliers needed for early stop.
     */
    double early_stop_min_inliers;

    /** \brief Visualize the inliers
     */
    bool visualize_ransac_inliers;

    /** \brief Use points migrated from other views into the current view (only
     * for localization)
     */
    bool use_migrated_points;

    /** \brief The minimum amount of inliers needed for a valid state
     * estimation.
     */
    int min_inliers;

    /** \brief Enable local optimisation flag. If distabled, the ransac module
     * simply calculates estimates from the test set, not the inlier set
     */
    bool enable_local_opt;

    /** \brief The number of parallel RANSAC threads
     */
    int num_threads;
  };

  RansacModule(std::string name = type_str_) : BaseModule{type_str_} {}

  /** \brief Given two frames and matches detects the inliers that fit the given
   * model, and provides an initial guess at transform T_q_m.
   */
  virtual void run(QueryCache &qdata, MapCache &mdata,
                   const std::shared_ptr<const Graph> &);

  void setConfig(std::shared_ptr<Config> &config);

 protected:
  /** \brief Visualization implementation
   */
  virtual void visualizeImpl(QueryCache &qdata, MapCache &mdata,
                             const std::shared_ptr<const Graph> &,
                             std::mutex &);

  /** \brief Generates a model for the RANSAC method. Subclass must override
   this.

   * \param[in] qdata, the reference frame. position of this frame is locked
   and set to the origin.
   * \param[in] mdata, The frame whose position is being optimized.
   * \return a pointer to the RANSAC model.
   */
  virtual std::shared_ptr<asrl::vision::SensorModelBase<Eigen::Matrix4d>>
  generateRANSACModel(QueryCache &qdata, MapCache &mdata) = 0;

  /** \brief Generates a sampler for the RANSAC method. Subclass must override
   * this.
   *
   * \param[in] qdata, the reference frame. position of this frame is locked
   * and set to the origin.
   * \param[in] mdata, The frame whose position is being optimized.
   * \return a pointer to the RANSAC model.
   */
  virtual std::shared_ptr<asrl::vision::BasicSampler> generateRANSACSampler(
      QueryCache &qdata, MapCache &mdata) = 0;

  /** \brief Virtual function, generates a filtered set of matches for the
   * RANSAC method.
   *
   * \param[in] qdata, query cache data.
   * \param[in] mdata, map cache data.
   * \return a filtered asrl::vision::RigMatches vector analogous to the
   * raw_matches vector
   */
  virtual std::vector<asrl::vision::RigMatches> generateFilteredMatches(
      QueryCache &qdata, MapCache &mdata);

  /** \brief offsets into the flattened map point structure, for each channel
   */
  OffsetMap map_channel_offsets_;

  /** \brief offsets into the flattened query point structure, for each channel
   */
  OffsetMap query_channel_offsets_;

  /** \brief Algorithm Configuration
   */
  std::shared_ptr<Config> config_;

 private:
  /** \brief flattens rig matches into a single vector of matches, while keep
   * track of channel offsets.
   *
   * \param The source matches
   * \param The destination matches
   */
  void flattenMatches(const asrl::vision::RigMatches &src_matches,
                      asrl::vision::SimpleMatches &dst_matches);

  /** \brief initialize a set of rig matches to mirror the structure of the
   * input rig matches
   *
   * \param The source matches
   * \param The destination matches
   */
  void mirrorStructure(const asrl::vision::RigMatches &src_matches,
                       asrl::vision::RigMatches &dst_matches);

  /** \brief initialize a set of rig matches to mirror the structure of theinput
   * rig matches
   *
   * \param The source matches
   * \param The destination matches
   */
  void inflateMatches(const asrl::vision::SimpleMatches &src_matches,
                      asrl::vision::RigMatches &dst_matches);
};

}  // namespace navigation
}  // namespace vtr
