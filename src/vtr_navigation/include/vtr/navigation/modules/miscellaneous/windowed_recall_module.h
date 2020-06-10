#pragma once

#include <vtr/navigation/modules/base_module.h>

namespace vtr {
namespace navigation {

/** \brief Reject outliers and estimate a preliminary transform
 */
class WindowedRecallModule : public BaseModule {
 public:
  /** \brief Module name
   */
  static constexpr auto type_str_ = "windowed_recall";

  /** \brief Module Configuration.
   */
  struct Config {
    int window_size;
  };

  /** \brief Default Constructor
   */
  WindowedRecallModule(std::string name = type_str_) : BaseModule{name} {};

  /** \brief Default Destructor
   */
  ~WindowedRecallModule() = default;

  /** \brief Given a window size, and a start vertex, recall all of the
   * landmarks and observations within the window, and set up a chain of poses
   * in a single coordinate frame.
   */
  virtual void run(QueryCache &qdata, MapCache &mdata,
                   const std::shared_ptr<const Graph> &graph);

  /** \brief Update the graph with the frame data for the live vertex
   */
  virtual void updateGraph(QueryCache &, MapCache &,
                           const std::shared_ptr<Graph> &, VertexId);

  /** \brief Sets the module's configuration.
   *
   * \param config the input configuration.
   */
  void setConfig(std::shared_ptr<Config> &config) { config_ = config; }

 private:
  /** \brief Loads a specific vertex's landmarks and observations into the
   * landmark and pose map.
   *
   * \param[in,out] lm_map A map containing all currently observed landmarks
   with observations.
   * \param[in,out] poses A map containing poses associated with each vertex.
   * \param current_vertex The current vertex
   * \param graph The pose graph.
   */
  void loadVertexData(LandmarkMap &lm_map, SteamPoseMap &poses,
                      SensorVehicleTransformMap &transforms,
                      const asrl::pose_graph::RCVertex::Ptr &current_vertex,
                      const std::string &rig_name,
                      const std::shared_ptr<const Graph> &graph);

  /** \brief Loads a all of the landmarks and observations for a specific
   * vertex's channel.
   *
   * \param[in,out] lm_map A map containing all currently observed landmarks
   with observations.
   * \param[in,out] poses A map containing poses associated with each vertex.
   * \param current_vertex The current vertex
   * \param graph The pose graph.
   */
  void loadLandmarksAndObs(
      LandmarkMap &lm_map, SteamPoseMap &poses,
      SensorVehicleTransformMap &transforms,
      const asrl::pose_graph::RCVertex::Ptr &current_vertex,
      asrl::vision_msgs::ChannelObservations *channel_obs,
      const std::string &rig_name, const std::shared_ptr<const Graph> &graph);

  /** \brief Given a set of vertices, computes poses for each vertex in a single
   * global coordinate frame.
   *
   * \param[in,out] poses A map containing poses associated with each vertex.
   * \param graph The pose graph.
   */
  void computePoses(SteamPoseMap &poses,
                    const std::shared_ptr<const Graph> &graph);

  void getTimesandVelocities(SteamPoseMap &poses,
                             const std::shared_ptr<const Graph> &graph);

  /** \brief Loads the sensor transform from robochunk via a vertex ID
   *
   * \param vid The Vertex ID of the vertex we need to load the transform from.
   * \param transforms The map of vertex ID to T_s_v's
   * \param rig_name the name of the current rig
   * \param graph A pointer to the pose graph.
   */
  void loadSensorTransform(const VertexId &vid,
                           SensorVehicleTransformMap &transforms,
                           const std::string &rig_name,
                           const Graph::ConstPtr &graph);

  /** \brief a map that keeps track of the pointers into the vertex landmark
   * messages.
   */
  std::map<VertexId, std::shared_ptr<asrl::vision_msgs::RigLandmarks>>
      vertex_landmarks_;

  /** \brief Algorithm Configuration
   */
  std::shared_ptr<Config> config_;
};

}  // namespace navigation
}  // namespace vtr
