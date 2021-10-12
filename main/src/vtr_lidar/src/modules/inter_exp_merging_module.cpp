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
 * \file inter_exp_merging_module.cpp
 * \brief InterExpMergingModule class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/inter_exp_merging_module.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

void InterExpMergingModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->depth = node->declare_parameter<int>(param_prefix + ".depth", config_->depth);
  config_->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config_->visualize);
  // clang-format on
}

void InterExpMergingModule::runImpl(QueryCache &qdata,
                                    const Graph::ConstPtr &) {
  if (!task_queue_) return;

  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    privileged_map_pub_ = qdata.node->create_publisher<PointCloudMsg>("inter_exp_merging_privileged", 5);
    bridging_map_pub_ = qdata.node->create_publisher<PointCloudMsg>("inter_exp_merging_bridging", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  if (qdata.live_id->isValid() &&
      qdata.live_id->minorId() >= (unsigned)config_->depth &&
      *qdata.keyframe_test_result == KeyframeTestResult::CREATE_VERTEX) {
    const auto target_vid =
        VertexId(qdata.live_id->majorId(),
                 qdata.live_id->minorId() - (unsigned)config_->depth);

    task_queue_->dispatch(std::make_shared<Task>(
        shared_from_base<InterExpMergingModule>(), config_, target_vid));
  }
}
void InterExpMergingModule::Task::run(const AsyncTaskExecutor::Ptr &,
                                      const Graph::Ptr &graph) {
  CLOG(INFO, "lidar.inter_exp_merging")
      << "Inter-Experience Merging for vertex: " << target_vid_;
  auto spatial_neighbors = graph->at(target_vid_)->spatialNeighbours();
  CLOG(WARNING, "lidar.inter_exp_merging")
      << "Spatial neighbors of " << target_vid_ << " are " << spatial_neighbors;
}

}  // namespace lidar
}  // namespace vtr