#pragma once

#include <asrl/messages/Landmarks.pb.h>
#include <asrl/messages/Observations.pb.h>
#include <robochunk_msgs/TimeStamp.pb.h>
#include <vtr/navigation/assemblies/base_assembly.h>

#include <asrl/vision/messages/bridge.hpp>

namespace asrl {
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
   *
   * \param qdata the query cache data.
   * \param mdata the map cache data.
   * \param graph the Spatio-Temporal Pose Graph.
   * \param live_id the vertex ID of the current pose.
   */
  virtual void updateGraph(QueryCache &qdata, MapCache &mdata,
                           const std::shared_ptr<Graph> &graph,
                           const VertexId &live_id);

 private:
  /** \brief Adds all candidate landmarks to the vertex as new landmarks.
   *
   * \param[in,out] landmarks the new landmarks message to be updated in the
   graph.
   * \param[in,out] observations the observations message to be updated in the
   graph.
   * \param rig_idx the index into the current rig.
   * \param qdata the query cache data.
   * \param mdata the map cache data.
   */
  void addAllLandmarks(vision_msgs::RigLandmarks &landmarks,
                       vision_msgs::RigObservations &observations,
                       const int &rig_idx, const QueryCache &qdata,
                       const MapCache &, const std::shared_ptr<Graph> &graph,
                       const graph_msgs::PersistentId &persistent_id);

  /** \brief Adds Observations for a specific channel
   *
   * \param[in,out] channel_obs the observations message to be updated in the
   graph.
   * \param channel_features The features corresponding to the candidate
   landmarks
   * \param channel_landmarks The candidate landmarks
   * \param live_id the vertex ID of the current pose.
   * \param rig_idx the index into the current rig.
   * \param channel_idx the index into the current channel.
   */
  void addChannelObs(vision_msgs::ChannelObservations *channel_obs,
                     const vision::ChannelFeatures &channel_features,
                     const vision::ChannelLandmarks &,
                     const graph_msgs::PersistentId &persistent_id,
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
   */
  void addLandmarksAndObs(vision_msgs::RigLandmarks &landmarks,
                          vision_msgs::RigObservations &observations,
                          const int &rig_idx, const QueryCache &qdata,
                          const MapCache &mdata, const std::shared_ptr<Graph> &,
                          const graph_msgs::PersistentId &persistent_id);

  /** \brief Adds Landmarks and Observations for new Feature Tracks.
   *
   * \param[in,out] new_landmarks the new landmarks message to be updated in the
   graph.
   * \param[in,out] new_observations the observations message to be updated in
   the graph.
   * \param new_landmark_flags flags every candidate landmark as new or not.
   * \param landmarks The candidate landmarks
   * \param features The features corresponding to the candidate landmarks
   * \param live_id the vertex ID of the current pose.
   * \param rig_idx the index into the current rig.
   * \param channel_idx the index into the current channel.
   */
  void addNewLandmarksAndObs(vision_msgs::ChannelLandmarks *new_landmarks,
                             vision_msgs::ChannelObservations *new_observations,
                             const std::vector<bool> &new_landmark_flags,
                             const vision::ChannelLandmarks &landmarks,
                             const vision::ChannelFeatures &features,
                             const graph_msgs::PersistentId &persistent_id,
                             const int &rig_idx, const int &channel_idx);

  /** \brief Adds Observations to landmarks in other vertices.
   *
   * \param[in,out] new_obs the observations message to be updated in the graph.
   * \param matches The matches between the query frame and old landmarks.
   * \param features The features corresponding to the landmarks
   * \param map_lm_obs The observations of previous landmarks.
   * \param new_landmark_flags Flags each candidate landmark as new or not.
   * \param live_id the vertex ID of the current pose.
   * \param rig_idx the index into the current rig.
   * \param channel_idx the index into the current channel.
   */
  void addObsToOldLandmarks(vision_msgs::ChannelObservations *new_obs,
                            const vision::SimpleMatches &matches,
                            const vision::ChannelFeatures &features,
                            const vision::ChannelObservations &map_lm_obs,
                            std::vector<bool> &new_landmark_flags,
                            const graph_msgs::PersistentId &persistent_id,
                            const int &rig_idx, const int &channel_idx);
  void updateLandmarks(vision_msgs::RigLandmarks &landmarks,
                       vision_msgs::RigObservations &observations,
                       const int &rig_idx, const QueryCache &qdata,
                       const MapCache &mdata, const std::shared_ptr<Graph> &,
                       const VertexId &live_id);
};

}  // namespace navigation
}  // namespace asrl
