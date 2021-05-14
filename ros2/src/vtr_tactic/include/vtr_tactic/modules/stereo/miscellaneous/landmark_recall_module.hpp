#pragma once

#include <vtr_tactic/modules/base_module.hpp>

namespace vtr {
namespace tactic {

/**
 * \brief A module that retrieves landmarks from a single graph vertex and
 * store them into map cache.
 * \details
 * requires:
 *   qdata.[rig_features, T_sensor_vehicle, *live_id]
 *   mdata.[*map_id]
 * outputs:
 *   mdata.[map_landmarks, T_sensor_vehicle_map, *map_id]
 *
 * For each landmark in the target keyframe, this module finds the first match
 * of the landmark, transforms that into the target keyframe, and stores the
 * transformed landmark into map_landmarks along with the descriptors, etc. It
 * does so for landmarks in all channels.
 * This module also retrieves the T_sensor_vehicle of any vertex that the
 * recalled landmark belongs to.
 * Need rig_name stored in qdata.rig_features to get the corresponding
 * T_sensor_vehicle
 */
class LandmarkRecallModule : public BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "landmark_recall";

  /** \brief Collection of config parameters */
  struct Config {
    /** \todo (Old) Filter visualization based on channel/camera? */
    std::string landmark_source;
    /**
     * \brief Whether to copy landmark matches (for thread safety, updated by
     * localization)
     */
    bool landmark_matches;
  };

  LandmarkRecallModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()) {}

  /**
   * \brief Sets the module's configuration.
   * \param config the input configuration.
   */
  void setConfig(std::shared_ptr<Config> &config) { config_ = config; }

 private:
  /**
   * \brief Given a target vertex (mdata.map_id), this module will recall all
   * landmarks observed in this vertex and compile them into a c++ structure
   * using Eigen and OpenCV data structures.
   * \param qdata The query data.
   * \param mdata The map data.
   * \param graph The Spatio Temporal Pose Graph.
   */
  void runImpl(QueryCache &qdata, MapCache &mdata,
               const Graph::ConstPtr &graph) override;

  /**
   * \brief Recalls landmarks for a specific rig and fills in the landmark
   * frame.
   * \param rig_name The name of the current rig.
   * \param map_id The vertex to recall the landmarks from.
   * \param graph A pointer to the pose graph.
   */
  LandmarkFrame recallLandmarks(const std::string &rig_name,
                                const VertexId &map_id,
                                const std::shared_ptr<const Graph> &graph);

  /**
   * \brief Recalls a landmark from the graph and adds it to the current
   * structure
   * \param[in,out] channel_lm the structure the landmark is to be added to.
   * \param landmark_obs the reference to the landmark.
   * \param landmark_idx the index into the landmark.
   * \param num_landmarks number of landmarks.
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

  /**
   * \brief Initializes the landmark structures memory.
   * \param[in,out] channel_lm The structure to be initialized.
   * \param num_landmarks the number of landmarks associated with this
   * structure.
   * \param desc_type the type of descriptors associated with these landmarks.
   */
  void initializeLandmarkMemory(
      vision::ChannelLandmarks &channel_lm, const uint32_t &num_landmarks,
      const vtr_messages::msg::DescriptorType &desc_type);

  /**
   * \brief Computes T_map_i (vehicle) for i in [landmark, map-1] and stores it
   * in the cache.
   * \param map_vid The Vertex ID of the vertex the point is to be squashed
   * into.
   * \param landmark_vid The VertexID of the vertex the point originiated from.
   * \param graph A pointer to the pose graph.
   * \throw runtime_error if a path between the two vertices does not exist.
   */
  lgmath::se3::Transformation cachedVehicleTransform(
      const VertexId &map_vid, const VertexId &landmark_vid,
      const Graph::ConstPtr &graph);

  /**
   * \brief Computes T_map_i (sensor) for i in [landmark, map-1] and stores it
   * in the cache.
   * \param map_vid The Vertex ID of the vertex the point is to be squashed
   * into.
   * \param landmark_vid The VertexID of the vertex the point originiated from.
   * \param graph A pointer to the pose graph.
   * \throw runtime_error if a path between the two vertices does not exist.
   */
  lgmath::se3::Transformation cachedSensorTransform(
      const VertexId &map_vid, const VertexId &landmark_vid,
      const Graph::ConstPtr &graph);

  /**
   * \brief Compounds the transforms between the two vertices. landmark_id The
   * VertexID of the vertex the point originiated from. map_id The Vertex ID of
   * the vertex the point is to be squashed into. graph A pointer to the pose
   * graph.
   * \return The transformation that takes points from the landmark frame into
   * the map frame.
   * \throw runtime_error if a path between the two vertices
   * does not exist.
   */
  Eigen::Vector3d squashPoint(const Eigen::Vector3d &point,
                              const VertexId &map_vid,
                              const VertexId &landmark_vid,
                              const Graph::ConstPtr &graph);

  /**
   * \brief Loads the sensor transform from robochunk via a vertex ID
   * landmark_id
   * \param rig_name the name of the current rig
   * \param[in] vid The Vertex ID of the vertex we need to load the transform
   * from.
   * \param[in] graph A pointer to the pose graph.
   */
  void loadSensorTransform(const VertexId &vid, const std::string &rig_name,
                           const Graph::ConstPtr &graph);

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;

  /**
   * \brief a map that keeps track of the pointers into the vertex landmark
   * messages.
   */
  std::map<VertexId, std::shared_ptr<vtr_messages::msg::RigLandmarks>>
      vertex_landmarks_;

  /**
   * \brief a map that keeps track of vehicle-frame transforms between vertex
   * ids.
   */
  typedef std::pair<VertexId, VertexId> TfCacheKey;
  typedef std::map<TfCacheKey, lgmath::se3::Transformation> TfCache;
  TfCache T_map_i_cache_;

  /**
   * \brief a map that keeps track of sensor-frame transforms between vertex
   * ids.
   */
  TfCache T_map_i_s_cache_;

  /** \brief the current step size of our landmark descriptor blob. */
  uint32_t step_size_;

  /** \brief a pointer into our landmark descriptor blob. */
  uint8_t *map_desc_ptr_;

  /**
   * \brief Transform that takes points from the vehicle frame to the sensor
   * frame.
   */
  EdgeTransform T_s_v_;
  std::map<VertexId, EdgeTransform> T_s_v_map_;
};

}  // namespace tactic
}  // namespace vtr
