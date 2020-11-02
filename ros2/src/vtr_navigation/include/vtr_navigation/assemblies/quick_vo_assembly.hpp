#pragma once

#include <vtr_messages/msg/rig_counts.hpp>
#include <vtr_messages/msg/rig_landmarks.hpp>
#include <vtr_messages/msg/rig_observations.hpp>
#include <vtr_messages/msg/velocity.hpp>
#include <vtr_navigation/assemblies/base_assembly.hpp>
#include <vtr_vision/messages/bridge.hpp>
#if false
#include <asrl/messages/Landmarks.pb.h>
#include <asrl/messages/Observations.pb.h>
#include <robochunk_msgs/TimeStamp.pb.h>
#endif

namespace vtr {
namespace navigation {

class QuickVoAssembly : public BaseAssembly {
 public:
  /** \brief An unique identifier for creating this assembly. */
  static constexpr auto type_str_ = "quick_vo";

  QuickVoAssembly() : BaseAssembly{type_str_} {}

  bool verify() const override;

  /**
   * \brief Localize the frame data against the map vertex using the (sub)graph
   */
  void run(QueryCache &qdata, MapCache &mdata,
           const std::shared_ptr<const Graph> &graph) override;

  /**
   * \brief Updates the newly added vertex with the results of Quick VO.
   * \details Updates the graph with the frame data for the live vertex.
   * \param qdata The query cache data.
   * \param mdata The map cache data.
   * \param graph The Spatio-Temporal Pose Graph.
   * \param live_id The vertex ID of the current pose.
   */
  void updateGraph(QueryCache &qdata, MapCache &mdata,
                   const std::shared_ptr<Graph> &graph,
                   const VertexId &live_id) override;

 private:
  /**
   * \brief Adds all candidate landmarks to the vertex as new landmarks.
   * \param[in,out] landmarks The new landmarks message to be updated in the
   * graph.
   * \param[in,out] observations The observations message to be updated in the
   * graph.
   * \param rig_idx The index into the current rig.
   * \param qdata The query cache data.
   * \param graph The STPG.
   * \param persistent_id TODO
   */
  void addAllLandmarks(
      vtr_messages::msg::RigLandmarks &landmarks,
      vtr_messages::msg::RigObservations &observations, const int &rig_idx,
      const QueryCache &qdata, const MapCache &,
      const std::shared_ptr<Graph> &graph,
      const vtr_messages::msg::GraphPersistentId &persistent_id);

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
  void addChannelObs(vtr_messages::msg::ChannelObservations &channel_obs,
                     const vision::ChannelFeatures &channel_features,
                     const vision::ChannelLandmarks &,
                     const vtr_messages::msg::GraphPersistentId &persistent_id,
                     const int &rig_idx, const int &channel_idx);

  /**
   * \brief Adds observations to previous landmarks and and adds new lanmarks.
   * \param[in,out] landmarks the new landmarks message to be updated in the
   graph.
   * \param[in,out] observations the observations message to be updated in the
   graph.
   * \param rig_idx the index into the current rig.
   * \param qdata the query cache data.
   * \param mdata the map cache data.
   * \param persistent_id TODO
   */
  void addLandmarksAndObs(
      vtr_messages::msg::RigLandmarks &landmarks,
      vtr_messages::msg::RigObservations &observations, const int &rig_idx,
      const QueryCache &qdata, const MapCache &mdata,
      const std::shared_ptr<Graph> &,
      const vtr_messages::msg::GraphPersistentId &persistent_id);

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
  void addNewLandmarksAndObs(
      vtr_messages::msg::ChannelLandmarks &new_landmarks,
      vtr_messages::msg::ChannelObservations &new_observations,
      const std::vector<bool> &new_landmark_flags,
      const vision::ChannelLandmarks &landmarks,
      const vision::ChannelFeatures &features,
      const vtr_messages::msg::GraphPersistentId &persistent_id,
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
  void addObsToOldLandmarks(
      vtr_messages::msg::ChannelObservations &new_obs,
      const vision::SimpleMatches &matches,
      const vision::ChannelFeatures &features,
      const vision::ChannelObservations &map_lm_obs,
      std::vector<bool> &new_landmark_flags,
      const vtr_messages::msg::GraphPersistentId &persistent_id,
      const int &rig_idx, const int &channel_idx);
#if false
  void updateLandmarks(vtr_messages::msg::RigLandmarks &landmarks,
                       vtr_messages::msg::RigObservations &observations,
                       const int &rig_idx, const QueryCache &qdata,
                       const MapCache &mdata, const std::shared_ptr<Graph> &,
                       const VertexId &live_id);
#endif
};
}  // namespace navigation
}  // namespace vtr
