// Copyright 2026, Autonomous Space Robotics Lab (ASRL)
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
 * \file direct_submap_similarity_test_module.hpp
 * \author Daniil Lisus, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_radar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace radar {

/**
 * \brief Decides whether to create a new submap based on how well the
 * current smoothed radar scan still matches the scan from the
 * last time a submap was created, once the two are aligned by the
 * accumulated odometry pose estimate between them.
 *
 * This is a drop-in alternative to tactic::SubmapTestModule
 * ("submap_distance_test"): rather than triggering purely on accumulated
 * translation/rotation, it triggers when a scaled cross-correlation
 * ("registration score", following DR-PoGO's loop-closure fine-registration
 * scoring) between the two dense images drops below a threshold, meaning
 * the current view has diverged too much from the reference submap scan to
 * keep relying on it (e.g. due to occlusion, structure change, or drift not
 * fully captured by the translation/rotation accumulators). The
 * translation/rotation thresholds are kept as a hard-cap fallback so that a
 * long feature-poor/self-similar stretch can't stall submap creation
 * indefinitely.
 */
class SubmapSimilarityTestModule : public tactic::BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "radar.submap_similarity_test";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    // Registration score (scaled cross-correlation between the current
    // smoothed scan and the reference scan warped into it by the
    // accumulated pose estimate) below which a new submap is created
    float min_registration_score = 0.5;

    // Hard-cap distance fallback
    float max_translation = 10.0;
    float max_rotation = 30.0;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  SubmapSimilarityTestModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {}

 private:
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  /**
   * \brief Warps reference_scan_ into current's pixel grid using T_cur_ref
   * (current frame relative to reference frame) and returns
   * sum(warped_reference .* current) / sum(reference_scan_^2), i.e. the
   * same scaled cross-correlation as fine_registration.py's
   * getRegistrationScore. Both current and the stored reference must
   * already be CV_32F, normalized to [0, 1].
   */
  double registrationScore_(const cv::Mat &current, double current_res,
                            const tactic::EdgeTransform &T_cur_ref) const;

  /** \brief Stores scan (converted to CV_32F/[0,1]) as the new reference. */
  void setReference_(const cv::Mat &scan, double scan_res);

  Config::ConstPtr config_;

  // Pose accumulated since the reference scan was captured, in the same
  // manner (and with the same semantics) as
  // tactic::SubmapTestModule::T_v_odo_submap_v_: T_current_reference, i.e.
  // maps a point expressed in the reference frame to the current frame.
  tactic::EdgeTransform T_v_odo_submap_v_ = tactic::EdgeTransform(true);

  cv::Mat reference_scan_;  // CV_32F, [0, 1], set at the last submap creation
  double reference_scan_res_ = 0.0;  // m/pixel of reference_scan_

  VTR_REGISTER_MODULE_DEC_TYPE(SubmapSimilarityTestModule);
};

}  // namespace radar
}  // namespace vtr
