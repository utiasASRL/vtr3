#pragma once

#include <vtr/navigation/modules/base_module.h>

namespace vtr {
namespace navigation {

/** \brief Migrate all landmarks found in the localization_map into a single
 * frame.
 */
class LandmarkMigrationModule : public BaseModule {
 public:
  /** \brief Module name
   */
  static constexpr auto type_str_ = "landmark_migration";

  /** \brief Module Configuration.
   */
  struct Config {};

  /** \brief Default Constructor
   */
  LandmarkMigrationModule(std::string name = type_str_) : BaseModule{name} {}

  ~LandmarkMigrationModule() = default;

  /** \brief Given a submap and target vertex located in this submap, this
   * module will transform all points into the coordinate frame of the target
   * vertex.
   *
   * \param qdata The query data.
   * \param mdata The map data.
   * \param graph The STPG.
   */
  virtual void run(QueryCache &qdata, MapCache &mdata,
                   const std::shared_ptr<const Graph> &graph);

  /** \brief Update the graph with the frame data for the live vertex
   */
  virtual void updateGraph(QueryCache &, MapCache &,
                           const std::shared_ptr<Graph> &, VertexId){};

  /** \brief Sets the module's configuration.
   *
   * \param config The input configuration.
   */
  void setConfig(std::shared_ptr<Config> &config) { config_ = config; }

 private:
  /** \brief Computes the transform that takes points from the current vertex to
   * the root vertex.
   *
   * \param qdata The query data
   * \param mdata The map data
   * \param curr ID of the current vertex.
   * \return T_curr_root The transform that takes points from the current vertex
   * to the root
   */
  EdgeTransform getTRootCurr(QueryCache &qdata, MapCache &mdata,
                             VertexId &curr);

  /** \brief Initializes the map data used in this module.
   *
   * \param mdata The map data
   */
  void initializeMapData(MapCache &mdata);

  /** \brief migrates landmarks from the current vertex to the root vertex.
   *
   * \param rig_idx the index into the current rig.
   * \param persist_id ID of the current vertex.
   * \param T_root_curr Transformation
   * \param mdata the Map data.
   * \param landmarks pointer to the landmarks.
   */
  void migrate(const int &rig_idx,
               const asrl::graph_msgs::PersistentId &persist_id,
               const EdgeTransform &T_root_curr, MapCache &mdata,
               std::shared_ptr<asrl::vision_msgs::RigLandmarks> &landmarks);

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

  /** \brief Algorithm Configuration
   */
  std::shared_ptr<Config> config_;
};

}  // namespace navigation
}  // namespace vtr
