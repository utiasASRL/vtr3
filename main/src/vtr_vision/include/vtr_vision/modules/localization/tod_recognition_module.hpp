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
 * \file tod_recognition_module.hpp
 * \brief TodRecognitionModule class definition
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_common/timing/utils.hpp>
#include <vtr_vision_msgs/msg/exp_recog_status.hpp>
#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_vision/cache.hpp>
#include "vtr_tactic/task_queue.hpp"
#include <vtr_vision/types.hpp>

namespace vtr {
namespace vision {

/**
 * \brief Recommend experiences based on time of day.
 * \details
 * Recommend experiences based on time of day.
 * requires:
 *   qdata.[live_id, localization_map, *recommended_experience]
 * outputs:
 *   qdata.[recommended_experience]
 */
class TodRecognitionModule : public tactic::BaseModule {
 public:
   PTR_TYPEDEFS(TodRecognitionModule);

  /** \brief Static module identifier. */
  static constexpr auto static_name = "timeofday_recognition";
  using time_point = common::timing::time_point;

  /** \brief Collection of config parameters */
  struct Config : public tactic::BaseModule::Config{
    PTR_TYPEDEFS(Config);

    /// Enables debugging logs.
    bool verbose = true;

    // INTERFACING //
    /// The number of experiences (including the privileged)
    /// that we should be recommended for localization.
    int num_exp = 4;
    /// Whether we should broadcast our recommendations (are we enabled).
    bool in_the_loop = false;

    // PARAMETERS //
    /// The weight to convert time-of-day difference to a distance
    float time_of_day_weight = 1.f;
    /// The weight to convert total time difference to a distance
    float total_time_weight = 1.f / 24.f;



    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

TodRecognitionModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {}

 private:
  /** \brief \todo */
      void run_(tactic::QueryCache &qdata, tactic::OutputCache &output, const tactic::Graph::Ptr &graph,
                const std::shared_ptr<tactic::TaskExecutor> &executor) override;


  // /** \brief \todo */
  void storeRunSelection(tactic::QueryCache &qdata,
                       const tactic::Graph::Ptr &graph,
                       tactic::VertexId vid, 
                       vtr_vision_msgs::msg::ExpRecogStatus status_msg);

  /** \brief Module configuration. */
  Config::ConstPtr config_;

  
  VTR_REGISTER_MODULE_DEC_TYPE(TodRecognitionModule);
};

/**
 * \brief Compute the time/time-of-day distance for the experiences in the
 * submap based on distance from a query point in time.
 * \param query_tp The query time point
 * \param submap The submap to precess
 * \param config COnfiguration (for weights)
 * \return The scored experiences
 */
ScoredRids scoreExperiences(const TodRecognitionModule::time_point &query_tp,
                            const tactic::GraphBase::Ptr &submap,
                            const TodRecognitionModule::Config &config);

}  // namespace vision
}  // namespace vtr
