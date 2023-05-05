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
 * \file template_pipeline.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_vision/cache.hpp"
#include "vtr_tactic/pipelines/base_pipeline.hpp"
#include "vtr_vision/modules/modules.hpp"
#include "vtr_vision/cache.hpp"

#include <vtr_common_msgs/msg/lie_group_transform.hpp>
#include <vtr_messages/msg/rig_counts.hpp>
#include <vtr_messages/msg/rig_landmarks.hpp>
#include <vtr_messages/msg/rig_observations.hpp>
#include <vtr_messages/msg/graph_persistent_id.hpp>
#include <vtr_messages/msg/velocity.hpp>

#include "steam.hpp"


namespace vtr {
namespace vision {


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



class StereoPipeline : public tactic::BasePipeline {
 public:
  PTR_TYPEDEFS(StereoPipeline);

  /** \brief Static pipeline identifier. */
  static constexpr auto static_name = "stereo";

  /** \brief Collection of config parameters */
  struct Config : public BasePipeline::Config {
    PTR_TYPEDEFS(Config);
    std::vector<std::string> preprocessing;
    std::vector<std::string> odometry;
    std::vector<std::string> localization;
    std::vector<std::string> bundle_adjustment;


    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  StereoPipeline(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name);
      //
      // : BasePipeline{module_factory, name}, config_(config)# {}

  virtual ~StereoPipeline();

  tactic::OutputCache::Ptr createOutputCache() const override;


  void initialize_(const tactic::OutputCache::Ptr &, const tactic::Graph::Ptr &) override {
    /// Perform necessary initialization of the pipeline, e.g., create and
    /// initialize modules.
    /// Pose-graph is given but may be an empty graph.
  }

  void preprocess_(const tactic::QueryCache::Ptr &qdata0, const tactic::OutputCache::Ptr &output0,
                   const tactic::Graph::Ptr &graph,
                   const std::shared_ptr<tactic::TaskExecutor> &executor) override; //{
    /// This method is called on every input data.
    /// The following will be in qdata:
    ///   - input data (raw)
    ///   - stamp: time stamp of this data.
    ///   - node: a shared pointer to ROS node that can be used to create
    ///   publishers for visualization
    /// Any processed data (e.g. features) should be put in qdata.
    /// This method should not touch the pose graph.
    /// Any data preprocessing module should not touch the pose graph.
  // }

  void runOdometry_(const tactic::QueryCache::Ptr &, const tactic::OutputCache::Ptr &,
                    const tactic::Graph::Ptr &,
                    const std::shared_ptr<tactic::TaskExecutor> &) override;
                        /// This method is called on every preprocessed input data.
    /// The following will be in qdata:
    ///   - everything from preprocessing.
    ///   - first_frame: the first data received for the current run (teach or
    ///   repeat).
    ///   - live_id: the current vertex being localized against for odometry, or
    ///   invalid if first frame.
    ///   - T_r_v_odo: odometry estimation from last input, or identity if this
    ///   is the first frame or a vertex has just been created.
    ///   - odo_success: whether or not odometry estimation is successful
    ///   - vertex_test_result: whether or not to create a new vertex,
    ///   always default to NO.
    /// This method should update the following:
    ///   - T_r_v_odo, odo_success, vertex_test_result
    /// This method should only read from the graph.
    /// Any debug info, extra stuff can be put in qdata.
  

  void runLocalization_(const tactic::QueryCache::Ptr &, const tactic::OutputCache::Ptr &,
                        const tactic::Graph::Ptr &,
                        const std::shared_ptr<tactic::TaskExecutor> &) override;
    /// This method is called in the following cases:
    ///   - first vertex of a teach that branches from existing path to
    ///   localize against the existing path (i.e., trunk)
    ///   - every frame when merging into existing graph to create a loop
    ///   - every frame when doing metric localization (e.g. before path
    ///   following)
    ///   - every vertex when repeating a path
    /// The following will be in qdata:
    ///   - everything from odometry and onVertexCreation.
    ///   - map_id: the vertex to be localized against by this method.
    ///   - T_r_v_loc: prior estimate from localization chain based on odometry.
    ///   - loc_success: whether or not localization is successful.
    /// This method should update the following:
    ///   - T_r_v_loc, loc_success
    /// This method may read from or write to the graph.
  

  // \todo Implement reset for multiple runs!
  // void reset();


  void onVertexCreation_(const tactic::QueryCache::Ptr &, const tactic::OutputCache::Ptr &,
                         const tactic::Graph::Ptr &,
                         const std::shared_ptr<tactic::TaskExecutor> &);

  void saveLandmarks(CameraQueryCache &qdata, const tactic::Graph::Ptr &graph,
                         const VertexId &live_id); 


  void addAllLandmarks(
      RigLandmarksMsg &landmarks, RigObservationsMsg &observations,
      const int &rig_idx, const CameraQueryCache &qdata, const tactic::Graph::Ptr &,
      const VertexId &persistent_id);


  void addChannelObs(
      ChannelObservationsMsg &channel_obs,
      const vision::ChannelFeatures &channel_features,
      const vision::ChannelLandmarks &, const VertexId &persistent_id,
      const int &rig_idx, const int &channel_idx);


  void addLandmarksAndObs(
      RigLandmarksMsg &landmarks, RigObservationsMsg &observations,
      const int &rig_idx, const CameraQueryCache &qdata, const tactic::Graph::Ptr &,
      const VertexId &persistent_id);



  void addNewLandmarksAndObs(ChannelLandmarksMsg &new_landmarks,
                             ChannelObservationsMsg &new_observations,
                             const std::vector<bool> &new_landmark_flags,
                             const vision::ChannelLandmarks &landmarks,
                             const vision::ChannelFeatures &features,
                             const VertexId &persistent_id,
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
                            const VertexId &persistent_id,
                            const int &rig_idx, const int &channel_idx);

  
  /**
   * \brief Updates the graph after the modules have run.
   * In the case of the localizer assembly, this includes updating the landmarks
   * in the live run to include matches to landmarks in the map.
   */
  void saveLocalization(CameraQueryCache &qdata,
                        const tactic::Graph::Ptr &graph,
                        const tactic::VertexId &live_id);

  void saveLocResults(CameraQueryCache &qdata, const tactic::Graph::Ptr &graph,
                      const tactic::VertexId &live_id);

      
//Carryover methods for internal pipeline use
private:
void setOdometryPrior(CameraQueryCache &, const tactic::Graph::Ptr &); 
tactic::EdgeTransform estimateTransformFromKeyframe(
    const tactic::Timestamp &kf_stamp, const tactic::Timestamp &curr_stamp,
    bool check_expiry);


 private:
  /** \brief Pipeline configuration */
  Config::ConstPtr config_;

  CameraQueryCache::Ptr candidate_qdata_ = nullptr;

  std::vector<tactic::BaseModule::Ptr> preprocessing_;
  std::vector<tactic::BaseModule::Ptr> odometry_;
  std::vector<tactic::BaseModule::Ptr> localization_;


  /**
   * \brief a pointer to a trjacetory estimate so that the transform can be
   * estimated at a future time
   */
  std::shared_ptr<steam::traj::const_vel::Interface> trajectory_;

  /** \brief Mutex to ensure thread safety with OpenCV HighGui calls */
  std::shared_ptr<std::mutex> vis_mutex_ptr_ = std::make_shared<std::mutex>();

  /** \brief \todo remove this mutex, no longer needed */
  std::shared_ptr<std::mutex> steam_mutex_ptr_ = std::make_shared<std::mutex>();

  tactic::Timestamp timestamp_odo_;

  VTR_REGISTER_PIPELINE_DEC_TYPE(StereoPipeline);
};

}  // namespace tactic
}  // namespace vtr
