#pragma once

#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_vision/messages/bridge.hpp>

namespace vtr {
namespace tactic {
namespace stereo {

/**
 * \brief A module that retrieves landmarks from multiple graph vertices and
 * store them into map cache.
 * \details
 */
class StereoWindowedRecallModule : public BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "stereo_windowed_recall";

  /** \brief Collection of config parameters */
  struct Config {
    int window_size;
  };

  StereoWindowedRecallModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()){};
  ~StereoWindowedRecallModule() = default;

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 private:
  /**
   * \brief Given a window size, and a start vertex, recall all of the
   * landmarks and observations within the window, and set up a chain of poses
   * in a single coordinate frame.
   */
  void runImpl(QueryCache &qdata, MapCache &mdata,
               const Graph::ConstPtr &graph) override;

  /** \brief Does nothing? */
  void updateGraphImpl(QueryCache &, MapCache &, const Graph::Ptr &, VertexId);

  /**
   * \brief Loads a specific vertex's landmarks and observations into the
   * landmark and pose map.
   * \param[in,out] lm_map A map containing all currently observed landmarks
   with observations.
   * \param[in,out] poses A map containing poses associated with each vertex.
   * \param transforms TODO
   * \param current_vertex The current vertex
   * \param rig_name TODO
   * \param graph The pose graph.
   */
  void loadVertexData(LandmarkMap &lm_map, SteamPoseMap &poses,
                      SensorVehicleTransformMap &transforms,
                      const pose_graph::RCVertex::Ptr &current_vertex,
                      const std::string &rig_name,
                      const GraphBase::ConstPtr &graph);

  /**
   * \brief Loads a all of the landmarks and observations for a specific
   * vertex's channel.
   * \param[in,out] lm_map A map containing all currently observed landmarks
   with observations.
   * \param[in,out] poses A map containing poses associated with each vertex.
   * \param transforms TODO
   * \param current_vertex The current vertex
   * \param channel_obs TODO
   * \param rig_name TODO
   * \param graph The pose graph.
   */
  void loadLandmarksAndObs(
      LandmarkMap &lm_map, SteamPoseMap &poses,
      SensorVehicleTransformMap &transforms,
      const pose_graph::RCVertex::Ptr &current_vertex,
      const vtr_messages::msg::ChannelObservations &channel_obs,
      const std::string &rig_name, const GraphBase::ConstPtr &graph);

  /**
   * \brief Given a set of vertices, computes poses for each vertex in a single
   * global coordinate frame.
   * \param[in,out] poses A map containing poses associated with each vertex.
   * \param graph The pose graph.
   */
  void computePoses(SteamPoseMap &poses, const GraphBase::ConstPtr &graph);

  void getTimesandVelocities(SteamPoseMap &poses,
                             const GraphBase::ConstPtr &graph);

  /**
   * \brief Loads the sensor transform from robochunk via a vertex ID
   * \param vid The Vertex ID of the vertex we need to load the transform from.
   * \param transforms The map of vertex ID to T_s_v's
   * \param rig_name the name of the current rig
   * \param graph A pointer to the pose graph.
   */
  void loadSensorTransform(const VertexId &vid,
                           SensorVehicleTransformMap &transforms,
                           const std::string &rig_name,
                           const GraphBase::ConstPtr &graph);

  /**
   * \brief a map that keeps track of the pointers into the vertex landmark
   * messages.
   */
  std::map<VertexId, std::shared_ptr<vtr_messages::msg::RigLandmarks>>
      vertex_landmarks_;

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;
};

}  // namespace stereo
}  // namespace tactic
}  // namespace vtr