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
 * \file experience_triage_module.hpp
 * \brief ExperienceTriageModule class definition
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_vision_msgs/msg/exp_recog_status.hpp>
#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_vision/cache.hpp>

namespace std {
/// Display the experience recognition status message in a readable way
std::ostream &operator<<(std::ostream &os,
                         const vtr_vision_msgs::msg::ExpRecogStatus &msg);
}  // namespace std

namespace vtr {
namespace vision {
using ExpRecogMsg = vtr_vision_msgs::msg::ExpRecogStatus;

/** \brief Given a subgraph, return the run ids present. */
tactic::RunIdSet getRunIds(const pose_graph::RCGraph::Base &subgraph);

/**
 * \brief Given a set of run ids, return those that are privileged (manual)
 * runs
 * \param graph The graph (has manual info)
 * \param rids The set of run ids to check
 */
tactic::RunIdSet privilegedRuns(const pose_graph::RCGraph::Base &graph,
                                tactic::RunIdSet rids);

/** \brief Given a subgraph, keep only vertices from runs in the mask */
pose_graph::RCGraph::Base::Ptr maskSubgraph(
    const pose_graph::RCGraph::Base::Ptr &graph, const tactic::RunIdSet &mask);

/**
 * \brief Fills up an experience recommendation set with n recommendations
 * \param recommends recommendation to fill
 * \param distance_rids run similarity scores
 * \param n the max number of runs in the recommendation
 * \return the newly inserted runs
 */
tactic::RunIdSet fillRecommends(tactic::RunIdSet *recommends,
                                const ScoredRids &distance_rids, unsigned n);

/**
 * \brief Mask the localization subgraph based on upstream experience (run)
 * recommendations
 * requires:
 *   qdata.[localization_map, *recommended_experiences, *localization_status]
 * outputs:
 *   qdata.[recommended_experiences, localization_status]
 */
class ExperienceTriageModule : public tactic::BaseModule {
 public:
  // template <class T>
  // using Ptr = std::shared_ptr<T>;
  PTR_TYPEDEFS(ExperienceTriageModule);

  /** \brief Static module identifier. */
  static constexpr auto static_name = "experience_triage";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config{
    PTR_TYPEDEFS(Config);
    bool verbose = false;
    bool always_privileged = true;
    bool only_privileged = false;
    bool in_the_loop = true;
    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                        const std::string &param_prefix);
  };



  ExperienceTriageModule(const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name) 
      : tactic::BaseModule{module_factory, name}, config_(config) {}

  /**
   * \brief Masks the localization map by the experience mask generated from
   * upstream recommenders
   * \param qdata Information about the live image
   * \param graph The spatio-temporal pose graph
   */
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output, const tactic::Graph::Ptr &graph,
                const std::shared_ptr<tactic::TaskExecutor> &executor) override;

  /**
   * \brief Saves the status message to the pose graph
   * \param qdata Information about the live image
   * \param graph The spatio-temporal pose graph
   * \param live_id The live vertex id
   */

  void updateGraphImpl(tactic::QueryCache &qdata,
                       const tactic::Graph::Ptr &graph,
                       tactic::VertexId id);

 private:
  /** \brief The status message to save to the graph */
  // vtr_vision_msgs::msg::ExpRecogStatus status_msg_;
  ExpRecogMsg status_msg_;

  /** \brief The module configuration */
  Config::ConstPtr config_;
  VTR_REGISTER_MODULE_DEC_TYPE(ExperienceTriageModule);
};

}  // namespace vision
}  // namespace vtr
