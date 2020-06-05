#pragma once

#include <asrl/navigation/modules/base_module.h>

namespace asrl {
namespace navigation {

/** \brief Reject outliers and estimate a preliminary transform
 */
class LandmarkRecallModule : public BaseModule {
 public:
  /** \brief Module name
   */
  static constexpr auto type_str_ = "landmark_recall";

  /** \brief Module Configuration.
   */
  struct Config {
    // TODO: Filter visualizationon based on channel/camera?
    std::string landmark_source;
    /// Whether to copy landmark matches (for thread safety, updated by
    /// localization)
    bool landmark_matches;
  };

  /** \brief Default Constructor
   */
  LandmarkRecallModule() = default;

  /** \brief Default Destructor
   */
  ~LandmarkRecallModule() = default;

  /** \brief Given a target vertex (mdata.map_id), this module will recall all
   * landmarks observed in this vertex and compile them into a c++ structure
   * using Eigen and OpenCV data structures.
   *
   * \param qdata The query data.
   * \param mdata The map data.
   * \param The Spatio Temporal Pose Graph.
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
  /** \brief Recalls landmarks for a specific rig and fills in the landmark
   * frame.
   *
   * \param[in,out] map_landmarks The Landmark frame to fill in.
   * \param rig_name The name of the current rig.
   * \param map_id The vertex to recall the landmarks from.
   * \param graph A pointer to the pose graph.
   */
  LandmarkFrame recallLandmarks(const std::string &rig_name,
                                const VertexId &map_id,
                                const std::shared_ptr<const Graph> &graph);

  /** \brief Recalls a landmark from the graph and adds it to the current
   * structure
   *
   * \param[in,out] channel_lm the structure the landmark is to be added to.
   * \param landmark_obs the reference to the landmark.
   * \param landmark_idx the index into the landmark.
   * \param rig_name the name of the current rig
   * \param map_id the vertex id of the current map vertex.
   * \param graph A pointer to the pose graph.
   */
  void recallLandmark(vision::ChannelLandmarks &channel_lm,
                      const vision::LandmarkMatch &landmark_obs,
                      const uint32_t &landmark_idx,
                      const uint32_t &num_landmarks,
                      const std::string &rig_name, const VertexId &map_id,
                      const std::shared_ptr<const Graph> &graph);

  /** \brief Initializes the landmark structures memory.
   *
   * \param[in,out] channel_lm The structure to be initialized.
   * \param num_landmarks the number of landmarks associated with this
   * structure.
   * \param desc_type the type of descriptors associated with these landmarks.
   */
  void initializeLandmarkMemory(vision::ChannelLandmarks &channel_lm,
                                const uint32_t &num_landmarks,
                                const vision_msgs::DescriptorType &desc_type);

  /** \brief Computes T_map_i (vehicle) for i in [landmark, map-1] and stores it
   * in the cache.
   *
   * \param landmark_id The VertexID of the vertex the point originiated from.
   * \param map_id The Vertex ID of the vertex the point is to be squashed into.
   * \param graph A pointer to the pose graph.
   * \throws runtime_error if a path between the two vertices does not exist.
   */
  lgmath::se3::Transformation cachedVehicleTransform(
      const VertexId &map_vid, const VertexId &landmark_vid,
      const Graph::ConstPtr &graph);

  /** \brief Computes T_map_i (sensor) for i in [landmark, map-1] and stores it
   * in the cache.
   *
   * \param landmark_id The VertexID of the vertex the point originiated from.
   * \param map_id The Vertex ID of the vertex the point is to be squashed into.
   * \param graph A pointer to the pose graph.
   * \throws runtime_error if a path between the two vertices does not exist.
   */
  lgmath::se3::Transformation cachedSensorTransform(
      const VertexId &map_vid, const VertexId &landmark_vid,
      const Graph::ConstPtr &graph);

  /** \brief Compounds the transforms between the two vertices. landmark_id The
   * VertexID of the vertex the point originiated from. map_id The Vertex ID of
   * the vertex the point is to be squashed into. graph A pointer to the pose
   * graph.
   *
   * \return The transformation that takes points from the landmark frame into
   * the map frame.
   * \throws runtime_error if a path between the two vertices
   * does not exist.
   */
  Eigen::Vector3d squashPoint(const Eigen::Vector3d &point,
                              const VertexId &map_vid,
                              const VertexId &landmark_vid,
                              const Graph::ConstPtr &graph);

  /** \brief Loads the sensor transform from robochunk via a vertex ID
   * landmark_id
   *
   * \param rig_name the name of the current rig
   *
   * The VertexID of the vertex the point originiated from vid. The Vertex ID of
   * the vertex we need to load the transform from graph A pointer to the pose
   * graph.
   */
  void loadSensorTransform(const VertexId &vid, const std::string &rig_name,
                           const Graph::ConstPtr &graph);

  /** \brief Algorithm Configuration
   */
  std::shared_ptr<Config> config_;

  /** \brief a map that keeps track of the pointers into the vertex landmark
   * messages.
   */
  std::map<VertexId, std::shared_ptr<vision_msgs::RigLandmarks>>
      vertex_landmarks_;

  /** \brief a map that keeps track of vehicle-frame transforms between vertex
   * ids.
   */
  typedef std::pair<VertexId, VertexId> TfCacheKey;
  typedef std::map<TfCacheKey, lgmath::se3::Transformation> TfCache;
  TfCache T_map_i_cache_;

  /** \brief a map that keeps track of sensor-frame transforms between vertex
   * ids.
   */
  TfCache T_map_i_s_cache_;

  /** \brief the current step size of our landmark descriptor blob.
   */
  uint32_t step_size_;

  /** \brief a pointer into our landmark descriptor blob.
   */
  uint8_t *map_desc_ptr_;

  /** \brief Transform that takes points from the vehicle frame to the sensor
   * frame.
   */
  EdgeTransform T_s_v_;
  std::map<VertexId, EdgeTransform> T_s_v_map_;
};

}  // namespace navigation
}  // namespace asrl
