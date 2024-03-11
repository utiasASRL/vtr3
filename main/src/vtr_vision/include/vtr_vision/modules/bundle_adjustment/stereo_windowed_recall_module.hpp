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
 * \file stereo_windowed_recall_module.hpp
 * \brief StereoWindowedRecallModule class definition
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_vision/cache.hpp>
#include <vtr_vision/types.hpp>
#include <vtr_vision/messages/bridge.hpp>
#include <vtr_common_msgs/msg/velocity.hpp>

#include "vtr_common/conversions/ros_lgmath.hpp"

namespace vtr {
namespace vision {
using VelocityMsg = vtr_common_msgs::msg::Velocity;


/**
 * \brief A module that retrieves landmarks from multiple graph vertices and
 * store them into map cache.
 * \details
 */
class StereoWindowedRecallModule : public tactic::BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "stereo_windowed_recall";
  PTR_TYPEDEFS(StereoWindowedRecallModule);

  /** \brief Collection of config parameters */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);
    
    int window_size = 1;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  StereoWindowedRecallModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {}

  ~StereoWindowedRecallModule() = default;


 private:
  /**
   * \brief Given a window size, and a start vertex, recall all of the
   * landmarks and observations within the window, and set up a chain of poses
   * in a single coordinate frame.
   */
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output, const tactic::Graph::Ptr &graph,
                const std::shared_ptr<tactic::TaskExecutor> &executor) override;


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
                      const tactic::GraphBase::ConstPtr &graph);

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
      const vtr_vision_msgs::msg::ChannelObservations &channel_obs,
      const std::string &rig_name, const tactic::GraphBase::ConstPtr &graph);

  /**
   * \brief Given a set of vertices, computes poses for each vertex in a single
   * global coordinate frame.
   * \param[in,out] poses A map containing poses associated with each vertex.
   * \param graph The pose graph.
   */
  void computePoses(SteamPoseMap &poses,
                    const tactic::GraphBase::ConstPtr &graph);

  void getTimesandVelocities(SteamPoseMap &poses,
                             const tactic::GraphBase::ConstPtr &graph);

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
                           const tactic::GraphBase::ConstPtr &graph);

  /**
   * \brief a map that keeps track of the pointers into the vertex landmark
   * messages.
   */
  std::map<tactic::VertexId, std::shared_ptr<vtr_vision_msgs::msg::RigLandmarks>>
      vertex_landmarks_;

  /** \brief Module configuration. */
  Config::ConstPtr config_;
  
  VTR_REGISTER_MODULE_DEC_TYPE(StereoWindowedRecallModule);
};

}  // namespace vision
}  // namespace vtr
