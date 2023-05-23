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
 * \file landmark_migration_module.hpp
 * \brief LandmarkMigrationModule class definition
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_vision/cache.hpp>
#include <vtr_vision/messages/bridge.hpp>

namespace vtr {
namespace vision {

/**
 * \brief Migrate all landmarks found in the localization_map into a single
 * frame.
 * \details
 * requires:
 *   qdata.[rig_features, rig_calibrations, T_sensor_vehicle
 *          localization_map, T_sensor_vehicle_map, map_id, localization_status
 *          T_r_m_prior]
 * outputs:
 *   qdata.[migrated_points, migrated_covariance, landmark_offset_map,
 *          migrated_landmark_ids, migrated_validity, migrated_points_3d,
 *          projected_map_points]
 */
class LandmarkMigrationModule : public tactic::BaseModule {
 public:
  PTR_TYPEDEFS(LandmarkMigrationModule);

  /** \brief Static module identifier. */
  static constexpr auto static_name = "landmark_migration";

  /** \brief Module Configuration. */

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config{
    PTR_TYPEDEFS(Config);
    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                        const std::string &param_prefix);
  };


  LandmarkMigrationModule(const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name) 
      : tactic::BaseModule{module_factory, name}, config_(config) {}
  

  /**
   * \brief Given a submap and target vertex located in this submap, this
   * module will transform all points into the coordinate frame of the target
   * vertex.
   * \param qdata The query data.
   * \param graph The STPG.
   */
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output, const tactic::Graph::Ptr &graph,
                const std::shared_ptr<tactic::TaskExecutor> &executor) override;

  /** \brief Update the graph with the frame data for the live vertex */
  void updateGraphImpl(tactic::QueryCache &qdata,
                       const tactic::Graph::Ptr &graph,
                       tactic::VertexId id);

 private:
  /**
   * \brief Computes the transform that takes points from the current vertex to
   * the root vertex.
   * \param qdata The query data
   * \param curr ID of the current vertex.
   * \return T_curr_root The transform that takes points from the current vertex
   * to the root
   */
  tactic::EdgeTransform getTRootCurr(CameraQueryCache &qdata,
                                     tactic::VertexId &curr);

  /** \brief Initializes the map data used in this module. */
  void initializeMapData(CameraQueryCache &qdata);

  /**
   * \brief migrates landmarks from the current vertex to the root vertex.
   * \param rig_idx the index into the current rig.
   * \param persist_id ID of the current vertex.
   * \param T_root_curr Transformation
   * \param qdata the query cache.
   * \param landmarks pointer to the landmarks.
   */
  void migrate(const int &rig_idx,
               const VertexId &persist_id,
               const tactic::EdgeTransform &T_root_curr,
               CameraQueryCache &qdata,
               std::shared_ptr<vtr_messages::msg::RigLandmarks> &landmarks);

  /**
   * \brief Loads the sensor transform from robochunk via a vertex ID
   * \param vid The Vertex ID of the vertex we need to load the transform from.
   * \param transforms The map of vertex ID to T_s_v's
   * \param rig_name the name of the current rig
   * \param graph A pointer to the pose graph.
   */
  void loadSensorTransform(const tactic::VertexId &vid,
                           SensorVehicleTransformMap &transforms,
                           const std::string &rig_name,
                           const tactic::Graph::ConstPtr &graph);

  /** \brief Algorithm Configuration */
  Config::ConstPtr config_;
  VTR_REGISTER_MODULE_DEC_TYPE(LandmarkMigrationModule);
};

}  // namespace vision
}  // namespace vtr
