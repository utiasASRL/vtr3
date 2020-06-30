#pragma once

#include <asrl/messages/Landmarks.pb.h>
#include <asrl/messages/Observations.pb.h>
#include <robochunk_msgs/TimeStamp.pb.h>
#include <vtr/navigation/assemblies/base_assembly.h>

#include <asrl/vision/messages/bridge.hpp>

namespace vtr {
namespace navigation {

class QuickVoAssembly : public BaseAssembly {
 public:
  /** \brief An unique identifier for creating this assembly.
   */
  static constexpr auto type_str_ = "quick_vo";

  QuickVoAssembly() : BaseAssembly{type_str_} {}

  bool verify() const;

  /** \brief Localize the frame data against the map vertex using the (sub)graph
   */
  virtual void run(QueryCache &qdata, MapCache &mdata,
                   const std::shared_ptr<const Graph> &graph);

  /** \brief Updates the newly added vertex with the results of Quick VO.
   * Updates the graph with the frame data for the live vertex.
   *
   * \param qdata The query cache data.
   * \param mdata The map cache data.
   * \param graph The Spatio-Temporal Pose Graph.
   * \param live_id The vertex ID of the current pose.
   */
  virtual void updateGraph(QueryCache &qdata, MapCache &mdata,
                           const std::shared_ptr<Graph> &graph,
                           const VertexId &live_id);

 private:
  /** \brief Adds all candidate landmarks to the vertex as new landmarks.
   *
   * \param[in,out] landmarks The new landmarks message to be updated in the
   * graph.
   * \param[in,out] observations The observations message to be updated in the
   * graph.
   * \param rig_idx The index into the current rig.
   * \param qdata The query cache data.
   * \param graph The STPG.
   * \param persistent_id TODO
   */
  void addAllLandmarks(asrl::vision_msgs::RigLandmarks &landmarks,
                       asrl::vision_msgs::RigObservations &observations,
                       const int &rig_idx, const QueryCache &qdata,
                       const MapCache &, const std::shared_ptr<Graph> &graph,
                       const asrl::graph_msgs::PersistentId &persistent_id);

  /** \brief Adds Observations for a specific channel
   *
   * \param[in,out] channel_obs the observations message to be updated in the
   graph.
   * \param channel_features The features corresponding to the candidate
   landmarks
   * \param persistent_id the vertex ID of the current pose.
   * \param rig_idx the index into the current rig.
   * \param channel_idx the index into the current channel.
   */
  void addChannelObs(asrl::vision_msgs::ChannelObservations *channel_obs,
                     const asrl::vision::ChannelFeatures &channel_features,
                     const asrl::vision::ChannelLandmarks &,
                     const asrl::graph_msgs::PersistentId &persistent_id,
                     const int &rig_idx, const int &channel_idx);

  /** \brief Adds observations to previous landmarks and and adds new lanmarks.
   *
   * \param[in,out] landmarks the new landmarks message to be updated in the
   graph.
   * \param[in,out] observations the observations message to be updated in the
   graph.
   * \param rig_idx the index into the current rig.
   * \param qdata the query cache data.
   * \param mdata the map cache data.
   * \param persistent_id TODO
   */
  void addLandmarksAndObs(asrl::vision_msgs::RigLandmarks &landmarks,
                          asrl::vision_msgs::RigObservations &observations,
                          const int &rig_idx, const QueryCache &qdata,
                          const MapCache &mdata, const std::shared_ptr<Graph> &,
                          const asrl::graph_msgs::PersistentId &persistent_id);

  /** \brief Adds Landmarks and Observations for new Feature Tracks.
   *
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
      asrl::vision_msgs::ChannelLandmarks *new_landmarks,
      asrl::vision_msgs::ChannelObservations *new_observations,
      const std::vector<bool> &new_landmark_flags,
      const asrl::vision::ChannelLandmarks &landmarks,
      const asrl::vision::ChannelFeatures &features,
      const asrl::graph_msgs::PersistentId &persistent_id, const int &rig_idx,
      const int &channel_idx);

  /** \brief Adds Observations to landmarks in other vertices.
   *
   * \param[in,out] new_obs the observations message to be updated in the graph.
   * \param matches The matches between the query frame and old landmarks.
   * \param features The features corresponding to the landmarks
   * \param map_lm_obs The observations of previous landmarks.
   * \param new_landmark_flags Flags each candidate landmark as new or not.
   * \param persistent_id the vertex ID of the current pose.
   * \param rig_idx the index into the current rig.
   * \param channel_idx the index into the current channel.
   */
  void addObsToOldLandmarks(asrl::vision_msgs::ChannelObservations *new_obs,
                            const asrl::vision::SimpleMatches &matches,
                            const asrl::vision::ChannelFeatures &features,
                            const asrl::vision::ChannelObservations &map_lm_obs,
                            std::vector<bool> &new_landmark_flags,
                            const asrl::graph_msgs::PersistentId &persistent_id,
                            const int &rig_idx, const int &channel_idx);
  void updateLandmarks(asrl::vision_msgs::RigLandmarks &landmarks,
                       asrl::vision_msgs::RigObservations &observations,
                       const int &rig_idx, const QueryCache &qdata,
                       const MapCache &mdata, const std::shared_ptr<Graph> &,
                       const VertexId &live_id);
};

}  // namespace navigation
}  // namespace vtr
