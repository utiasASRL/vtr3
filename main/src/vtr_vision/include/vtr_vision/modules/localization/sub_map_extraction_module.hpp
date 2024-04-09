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
 * \file sub_map_extraction_module.hpp
 * \brief SubMapExtractionModule class definition
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_vision/cache.hpp>

namespace vtr {
namespace vision {

/**
 * \brief
 * \details
 * requires:
 *   qdata.[live_id, map_id, T_q_m_prior, *recommended_experience]
 * outputs:
 *   qdata.[localization_map, localization_status]
 */
class SubMapExtractionModule : public tactic::BaseModule {
 public:
  PTR_TYPEDEFS(SubMapExtractionModule);

  /** \brief Static module identifier. */
  static constexpr auto static_name = "sub_map_extraction";

  /** \brief Collection of config parameters */
  struct Config : public tactic::BaseModule::Config{
    PTR_TYPEDEFS(Config);
    /** \brief Minimum depth of the sub map, in the temporal direction */
    int temporal_min_depth = 5;

    /** \brief Maximum depth of the submap in the temporal direction */
    int temporal_max_depth = 10;

    /** \brief Whether or not to search in the spatial direction. */
    bool search_spatially = true;

    /** \brief Search temporally for vertices within sigma_scale*sigma */
    double sigma_scale = 3.0;

    /** \brief The angle weight to apply to the depth */
    double angle_weight = 5.0;
    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  SubMapExtractionModule(const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name) 
      : tactic::BaseModule{module_factory, name}, config_(config) {}


 private:
  /**
   * \brief Given a target vertex, this module will build a subgraph centered
   * on this vertex containing neighboring vertices up to a temporal depth, and
   * spatial neighbors not in the live run.
   * \param qdata The query data.
   * \param graph The Spatio Temporal Pose Graph.
   */
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output, const tactic::Graph::Ptr &graph,
                const std::shared_ptr<tactic::TaskExecutor> &executor) override;


  /**
   * \brief Extract submap with a temporal depth and optional spatial neighbours
   * \param graph The spatio temporal pose graph
   * \param root The vertex id we're expanding from for the submap
   * \param current_run the live run that we don't want in the submap
   * \param mask TODO
   * \param temporal_depth TODO
   * \param spatial_neighbours TODO
   */
  static tactic::GraphBase::Ptr extractSubmap(
      const tactic::Graph &graph, const tactic::VertexId &root,
      uint32_t current_run, tactic::RunIdSet *mask, int temporal_depth,
      bool spatial_neighbours);

 private:
  /**
   * \brief calculates the depth of the localization map given the current
   * lateral uncertainty.
   * \param root The vertex the map is centered on.
   * \param lateral_uncertainty The 1-Sigma uncertainty in the x-axis.
   * \param graph The graph.
   */
  int calculateDepth(tactic::VertexId root, double lateral_uncertainty,
                     const tactic::Graph::ConstPtr &graph);

  /** \brief Algorithm Configuration */
  Config::ConstPtr config_;
  VTR_REGISTER_MODULE_DEC_TYPE(SubMapExtractionModule);

};

}  // namespace vision
}  // namespace vtr
