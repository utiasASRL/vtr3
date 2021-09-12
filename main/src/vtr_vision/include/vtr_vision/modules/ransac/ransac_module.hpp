// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file ransac_module.hpp
 * \brief RansacModule class definition
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_vision/cache.hpp>
#include <vtr_vision/outliers.hpp>
#include <vtr_vision/outliers/sampler/basic_sampler.hpp>
#include <vtr_vision/sensors/sensor_model_base.hpp>
#include <vtr_vision/visualize.hpp>

namespace vtr {
namespace vision {

/** \brief The base RANSAC module. */
class RansacModule : public tactic::BaseModule {
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
    bool enable = true;

    /** \brief Max iterations for RANSAC. */
    int iterations = 2000;

    /** \brief Type of RANSAC to use. Options [ Vanilla ]. */
    std::string flavor = "Vanilla";

    /** \brief Std. deviation of the keypoint uncertainty, in pixels. */
    double sigma = 3.5;

    /** \brief Threshold on inlier, in std. deviations. */
    double threshold = 5.0;

    /** \brief Minimum ratio of inliers to outliers needed for early stop. */
    double early_stop_ratio = 1.0;

    /** \brief Minimum number of inliers needed for early stop. */
    int early_stop_min_inliers = 200;

    /** \brief Visualize the inliers */
    bool visualize_ransac_inliers = true;

    /**
     * \brief Use points migrated from other views into the current view (only
     * for localization)
     */
    bool use_migrated_points = false;

    /**
     * \brief The minimum amount of inliers needed for a valid state estimation.
     */
    int min_inliers = 15;

    /**
     * \brief Enable local optimisation flag. If distabled, the ransac module
     * simply calculates estimates from the test set, not the inlier set
     */
    bool enable_local_opt;

    /** \brief The number of parallel RANSAC threads */
    int num_threads;
  };

  RansacModule(const std::string &name = static_name)
      : tactic::BaseModule{name}, config_(std::make_shared<Config>()){};

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 private:
  /**
   * \brief Given two frames and matches detects the inliers that fit the given
   * model, and provides an initial guess at transform T_q_m.
   */
  void runImpl(tactic::QueryCache &qdata,
               const tactic::Graph::ConstPtr &) override;

  /** \brief Visualization implementation */
  void visualizeImpl(tactic::QueryCache &qdata,
                     const tactic::Graph::ConstPtr &) override;

  /**
   * \brief Generates a model for the RANSAC method. Subclass must override
   this.
   * \param[in] qdata the reference frame. position of this frame is locked
   and set to the origin.
   * \return a pointer to the RANSAC model.
   */
  virtual std::shared_ptr<SensorModelBase<Eigen::Matrix4d>> generateRANSACModel(
      CameraQueryCache &qdata) = 0;

  /**
   * \brief Generates a sampler for the RANSAC method. Subclass must override
   * this.
   * \param[in] qdata the reference frame. position of this frame is locked
   * and set to the origin.
   * \return a pointer to the RANSAC model.
   */
  virtual std::shared_ptr<BasicSampler> generateRANSACSampler(
      CameraQueryCache &qdata) = 0;

  /**
   * \brief Generates a filtered set of matches for the RANSAC method.
   * \param[in] qdata query cache data.
   * \return a filtered RigMatches vector analogous to the
   * raw_matches vector
   */
  virtual std::vector<RigMatches> generateFilteredMatches(
      CameraQueryCache &qdata);

 protected:
  /** \brief Algorithm Configuration */
  std::shared_ptr<Config> config_;

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
  void flattenMatches(const RigMatches &src_matches,
                      SimpleMatches &dst_matches);

  /**
   * \brief initialize a set of rig matches to mirror the structure of the
   * input rig matches
   * \param src_matches The source matches
   * \param dst_matches The destination matches
   */
  void mirrorStructure(const RigMatches &src_matches, RigMatches &dst_matches);

  /**
   * \brief initialize a set of rig matches to mirror the structure of theinput
   * rig matches
   * \param src_matches The source matches
   * \param dst_matches The destination matches
   */
  void inflateMatches(const SimpleMatches &src_matches,
                      RigMatches &dst_matches);
};

}  // namespace vision
}  // namespace vtr
