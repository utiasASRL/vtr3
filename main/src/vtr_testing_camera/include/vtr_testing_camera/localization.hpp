#pragma once

#include <vtr_testing_camera/offline.hpp>

#include <vtr_common/timing/time_utils.hpp>
#include <vtr_common/utils/filesystem.hpp>
#include <vtr_logging/logging_init.hpp>
#include <vtr_navigation/navigator.hpp>
#include <vtr_pose_graph/evaluator/common.hpp>

using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::navigation;
using namespace vtr::pose_graph;

using LocEvaluator = eval::Mask::Privileged<RCGraph>::Caching;

class LocalizationNavigator : public OfflineNavigator {
 public:
  LocalizationNavigator(const rclcpp::Node::SharedPtr node,
                        std::string output_dir)
      : OfflineNavigator(node, output_dir) {
    // Get the path that we should repeat
    VertexId::Vector sequence;
    sequence.reserve(graph_->numberOfVertices());
    LOG(INFO) << "Number of Vertices: " << graph_->numberOfVertices();
    // Extract the privileged sub graph from the full graph.
    LocEvaluator::Ptr evaluator(new LocEvaluator());
    evaluator->setGraph(graph_.get());
    auto privileged_path = graph_->getSubgraph(0ul, evaluator);
    for (auto it = privileged_path->begin(0ul); it != privileged_path->end();
         ++it) {
      LOG(INFO) << it->v()->id();
      sequence.push_back(it->v()->id());
    }

    // normally the state machine would add a run when a goal is started. We
    // spoof that here.
    tactic_->addRun();

    // Create a branch pipeline.
    tactic_->setPipeline(tactic::PipelineMode::Following);

    tactic_->setPath(sequence);
  }

 private:
};