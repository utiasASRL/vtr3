#pragma once

#include <vtr_common/timing/time_utils.hpp>
#include <vtr_common/utils/macros.hpp>
#include <vtr_lgmath_extensions/conversions.hpp>
#include <vtr_logging/logging.hpp>  // for debugging only
#include <vtr_tactic/caches.hpp>
#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_tactic/modules/module_factory.hpp>
#include <vtr_tactic/pipelines/base_pipeline.hpp>
#include <vtr_tactic/types.hpp>

#include <vtr_messages/msg/rig_counts.hpp>
#include <vtr_messages/msg/rig_landmarks.hpp>
#include <vtr_messages/msg/rig_observations.hpp>
#include <vtr_messages/msg/velocity.hpp>

//
using TimeStampMsg = vtr_messages::msg::TimeStamp;

using RigLandmarksMsg = vtr_messages::msg::RigLandmarks;
using RigObservationsMsg = vtr_messages::msg::RigObservations;
using RigCountsMsg = vtr_messages::msg::RigCounts;

using ChannelObservationsMsg = vtr_messages::msg::ChannelObservations;
using ChannelFeaturesMsg = vtr_messages::msg::ChannelFeatures;
using ChannelLandmarksMsg = vtr_messages::msg::ChannelLandmarks;

using TransformMsg = vtr_messages::msg::Transform;
using VelocityMsg = vtr_messages::msg::Velocity;
using ImageMsg = vtr_messages::msg::Image;
using GraphPersistentIdMsg = vtr_messages::msg::GraphPersistentId;

namespace vtr {
namespace tactic {

class StereoPipeline : public BasePipeline {
 public:
  using Ptr = std::shared_ptr<StereoPipeline>;

  /** \brief Static pipeline identifier. */
  static constexpr auto static_name = "stereo";

  /** \brief Collection of config parameters */
  struct Config {
    std::vector<std::string> preprocessing;
    std::vector<std::string> odometry;
    std::vector<std::string> bundle_adjustment;
    std::vector<std::string> localization;
  };

  StereoPipeline(const std::string &name = static_name) : BasePipeline{name} {}

  virtual ~StereoPipeline() {}

  void setConfig(std::shared_ptr<Config> &config) { config_ = config; }

  void initialize(const Graph::Ptr &graph) override;

  void preprocess(QueryCache::Ptr &qdata, const Graph::Ptr &graph) override;

  void runOdometry(QueryCache::Ptr &qdata, const Graph::Ptr &graph) override;

  void visualizeOdometry(QueryCache::Ptr &qdata,
                         const Graph::Ptr &graph) override;

  void runLocalization(QueryCache::Ptr &qdata,
                       const Graph::Ptr &graph) override;

  void visualizeLocalization(QueryCache::Ptr &qdata,
                             const Graph::Ptr &graph) override;

  void processKeyframe(QueryCache::Ptr &qdata, const Graph::Ptr &graph,
                       VertexId live_id) override;

  void waitForKeyframeJob() override;

 private:
  void runBundleAdjustment(QueryCache::Ptr qdata, const Graph::Ptr graph,
                           VertexId live_id);

  void saveLandmarks(QueryCache &qdata, const Graph::Ptr &graph,
                     const VertexId &live_id);

  void setOdometryPrior(QueryCache::Ptr &qdata, const Graph::Ptr &graph);

  EdgeTransform estimateTransformFromKeyframe(const TimeStampMsg &kf_stamp,
                                              const TimeStampMsg &curr_stamp,
                                              bool check_expiry);

  /**
   * \brief Adds all candidate landmarks to the vertex as new landmarks.
   * \param[in,out] landmarks The new landmarks message to be updated in the
   * graph.
   * \param[in,out] observations The observations message to be updated in
   * the graph. \param rig_idx The index into the current rig. \param qdata
   * The query cache data. \param graph The STPG. \param persistent_id TODO
   */
  void addAllLandmarks(RigLandmarksMsg &landmarks,
                       RigObservationsMsg &observations, const int &rig_idx,
                       const QueryCache &qdata,
                       const std::shared_ptr<Graph> &graph,
                       const GraphPersistentIdMsg &persistent_id);

  /**
   * \brief Adds Observations for a specific channel
   * \param[in,out] channel_obs the observations message to be updated in the
   graph.
   * \param channel_features The features corresponding to the candidate
   landmarks
   * \param persistent_id the vertex ID of the current pose.
   * \param rig_idx the index into the current rig.
   * \param channel_idx the index into the current channel.
   */
  void addChannelObs(ChannelObservationsMsg &channel_obs,
                     const vision::ChannelFeatures &channel_features,
                     const vision::ChannelLandmarks &,
                     const GraphPersistentIdMsg &persistent_id,
                     const int &rig_idx, const int &channel_idx);

  /**
   * \brief Adds observations to previous landmarks and and adds new lanmarks.
   * \param[in,out] landmarks the new landmarks message to be updated in the
   graph.
   * \param[in,out] observations the observations message to be updated in the
   graph.
   * \param rig_idx the index into the current rig.
   * \param qdata the query cache data.
   * \param persistent_id TODO
   */
  void addLandmarksAndObs(RigLandmarksMsg &landmarks,
                          RigObservationsMsg &observations, const int &rig_idx,
                          const QueryCache &qdata,
                          const std::shared_ptr<Graph> &,
                          const GraphPersistentIdMsg &persistent_id);

  /**
   * \brief Adds Landmarks and Observations for new Feature Tracks.
   * \param[in,out] new_landmarks the new landmarks message to be updated in the
   graph.
   * \param[in,out] new_observations the observations message to be updated in
   the graph.
   * \param new_landmark_flags flags every candidate landmark as new or not.
   * \param landmarks The candidate landmarks
   * \param features The features corresponding to the candidate landmarks
   * \param persistent_id the vertex ID of the current pose.
   * \param rig_idx the index into the current rig.
   * \param channel_idx the index into the current channel.
   */
  void addNewLandmarksAndObs(ChannelLandmarksMsg &new_landmarks,
                             ChannelObservationsMsg &new_observations,
                             const std::vector<bool> &new_landmark_flags,
                             const vision::ChannelLandmarks &landmarks,
                             const vision::ChannelFeatures &features,
                             const GraphPersistentIdMsg &persistent_id,
                             const int &rig_idx, const int &channel_idx);

  /**
   * \brief Adds Observations to landmarks in other vertices.
   * \param[in,out] new_obs the observations message to be updated in the graph.
   * \param matches The matches between the query frame and old landmarks.
   * \param features The features corresponding to the landmarks
   * \param map_lm_obs The observations of previous landmarks.
   * \param new_landmark_flags Flags each candidate landmark as new or not.
   * \param persistent_id the vertex ID of the current pose.
   * \param rig_idx the index into the current rig.
   * \param channel_idx the index into the current channel.
   */
  void addObsToOldLandmarks(ChannelObservationsMsg &new_obs,
                            const vision::SimpleMatches &matches,
                            const vision::ChannelFeatures &features,
                            const vision::ChannelObservations &map_lm_obs,
                            std::vector<bool> &new_landmark_flags,
                            const GraphPersistentIdMsg &persistent_id,
                            const int &rig_idx, const int &channel_idx);

  /**
   * \brief Updates the graph after the modules have run.
   * In the case of the localizer assembly, this includes updating the landmarks
   * in the live run to include matches to landmarks in the map.
   */
  void saveLocalization(QueryCache &qdata, const Graph::Ptr &graph,
                        const VertexId &live_id);

  void saveLocResults(QueryCache &qdata, const Graph::Ptr &graph,
                      const VertexId &live_id);

 private:
  /** \brief Pipeline configuration */
  std::shared_ptr<Config> config_ = std::make_shared<Config>();

  // cache
  /// \todo this will cause trouble when paralellize keyframe creation
  QueryCache::Ptr candidate_qdata_ = nullptr;

  std::mutex bundle_adjustment_mutex_;
  std::future<void> bundle_adjustment_thread_future_;

  std::vector<BaseModule::Ptr> preprocessing_;
  std::vector<BaseModule::Ptr> odometry_;
  std::vector<BaseModule::Ptr> bundle_adjustment_;
  std::vector<BaseModule::Ptr> localization_;

  /**
   * \brief a pointer to a trjacetory estimate so that the transform can be
   * estimated at a future time
   */
  std::shared_ptr<steam::se3::SteamTrajInterface> trajectory_;
  /** \brief the time at which the trajectory was estimated */
  common::timing::time_point trajectory_time_point_;
};

}  // namespace tactic
}  // namespace vtr
