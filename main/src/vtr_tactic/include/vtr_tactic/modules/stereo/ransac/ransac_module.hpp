#pragma once

#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_tactic/visualize.hpp>
#include <vtr_vision/outliers.hpp>
#include <vtr_vision/outliers/sampler/basic_sampler.hpp>
#include <vtr_vision/sensors/sensor_model_base.hpp>

namespace vtr {
namespace tactic {

/** \brief The base RANSAC module. */
class RansacModule : public BaseModule {
 public:
  using OffsetMap = std::map<uint32_t, std::pair<uint32_t, uint32_t>>;

  /**
   * \brief Static module identifier.
   * \todo change this to static_name
   */
  static constexpr auto static_name = "ransac";

  /** \brief Collection of config parameters */
  struct Config {
    /**
     * \brief Ransac enable flag. If disabled, the ransac module simply serves
     * as a passthrough
     */
    bool enable;

    /** \brief Max iterations for RANSAC. */
    int iterations;

    /** \brief Type of RANSAC to use. Options [ Vanilla ]. */
    std::string flavor;

    /** \brief Std. deviation of the keypoint uncertainty, in pixels. */
    double sigma;

    /** \brief Threshold on inlier, in std. deviations. */
    double threshold;

    /** \brief Minimum ratio of inliers to outliers needed for early stop. */
    double early_stop_ratio;

    /** \brief Minimum number of inliers needed for early stop. */
    int early_stop_min_inliers;

    /** \brief Visualize the inliers */
    bool visualize_ransac_inliers;

    /**
     * \brief Use points migrated from other views into the current view (only
     * for localization)
     */
    bool use_migrated_points;

    /**
     * \brief The minimum amount of inliers needed for a valid state estimation.
     */
    int min_inliers;

    /**
     * \brief Enable local optimisation flag. If distabled, the ransac module
     * simply calculates estimates from the test set, not the inlier set
     */
    bool enable_local_opt;

    /** \brief The number of parallel RANSAC threads */
    int num_threads;
  };

  RansacModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()){};

  void setConfig(std::shared_ptr<Config> &config) { config_ = config; }

 private:
  /**
   * \brief Given two frames and matches detects the inliers that fit the given
   * model, and provides an initial guess at transform T_q_m.
   */
  void runImpl(QueryCache &qdata, MapCache &mdata,
               const Graph::ConstPtr &) override;

  /** \brief Visualization implementation */
  void visualizeImpl(QueryCache &qdata, MapCache &mdata,
                     const Graph::ConstPtr &, std::mutex &) override;

  /**
   * \brief Generates a model for the RANSAC method. Subclass must override
   this.
   * \param[in] qdata the reference frame. position of this frame is locked
   and set to the origin.
   * \param[in] mdata The frame whose position is being optimized.
   * \return a pointer to the RANSAC model.
   */
  virtual std::shared_ptr<vision::SensorModelBase<Eigen::Matrix4d>>
  generateRANSACModel(QueryCache &qdata, MapCache &mdata) = 0;

  /**
   * \brief Generates a sampler for the RANSAC method. Subclass must override
   * this.
   * \param[in] qdata the reference frame. position of this frame is locked
   * and set to the origin.
   * \param[in] mdata The frame whose position is being optimized.
   * \return a pointer to the RANSAC model.
   */
  virtual std::shared_ptr<vision::BasicSampler> generateRANSACSampler(
      QueryCache &qdata, MapCache &mdata) = 0;

  /**
   * \brief Generates a filtered set of matches for the RANSAC method.
   * \param[in] qdata query cache data.
   * \param[in] mdata map cache data.
   * \return a filtered vision::RigMatches vector analogous to the
   * raw_matches vector
   */
  virtual std::vector<vision::RigMatches> generateFilteredMatches(
      QueryCache &qdata, MapCache &mdata);

  /** \brief Algorithm Configuration */
  std::shared_ptr<Config> config_;

 protected:
  /** \brief offsets into the flattened map point structure, for each channel */
  OffsetMap map_channel_offsets_;

  /**
   * \brief offsets into the flattened query point structure, for each channel
   */
  OffsetMap query_channel_offsets_;

 private:
  /**
   * \brief flattens rig matches into a single vector of matches, while keep
   * track of channel offsets.
   * \param src_matches The source matches
   * \param dst_matches The destination matches
   */
  void flattenMatches(const vision::RigMatches &src_matches,
                      vision::SimpleMatches &dst_matches);

  /**
   * \brief initialize a set of rig matches to mirror the structure of the
   * input rig matches
   * \param src_matches The source matches
   * \param dst_matches The destination matches
   */
  void mirrorStructure(const vision::RigMatches &src_matches,
                       vision::RigMatches &dst_matches);

  /**
   * \brief initialize a set of rig matches to mirror the structure of theinput
   * rig matches
   * \param src_matches The source matches
   * \param dst_matches The destination matches
   */
  void inflateMatches(const vision::SimpleMatches &src_matches,
                      vision::RigMatches &dst_matches);
};

}  // namespace tactic
}  // namespace vtr
