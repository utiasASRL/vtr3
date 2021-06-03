#include "rclcpp/rclcpp.hpp"

#include <vtr_common/timing/time_utils.hpp>
#include <vtr_common/utils/filesystem.hpp>
#include <vtr_logging/logging_init.hpp>
#include <vtr_navigation/navigator.hpp>
#include <vtr_pose_graph/evaluator/common.hpp>

using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::navigation;
using namespace vtr::pose_graph;
using namespace vtr::tactic;

using LocEvaluator = eval::Mask::Privileged<RCGraph>::Caching;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("navigator");

  /// Log into a subfolder of the data directory (if requested to log)
  auto data_dir_str = node->declare_parameter<std::string>("data_dir", "/tmp");
  auto clear_data_dir = node->declare_parameter<bool>("clear_data_dir", false);
  fs::path data_dir{utils::expand_user(utils::expand_env(data_dir_str))};
  if (clear_data_dir) fs::remove_all(data_dir);
  auto to_file = node->declare_parameter<bool>("log_to_file", false);
  std::string log_filename;
  if (to_file) {
    auto log_name = timing::toIsoFilename(timing::clock::now());
    log_filename = data_dir / "logs" / (log_name + ".log");
  }
  configureLogging(log_filename, false);
  LOG_IF(to_file, INFO) << "Logging to: " << log_filename;
  LOG_IF(!to_file, WARNING) << "NOT LOGGING TO A FILE.";

  LOG(INFO) << "Starting the Navigator node. Hello!";
  Navigator navigator{node};

  /// Localization specific

  // Get the path that we should repeat
  VertexId::Vector sequence;
  sequence.reserve(navigator.graph()->numberOfVertices());
  LOG(INFO) << "Number of Vertices: " << navigator.graph()->numberOfVertices();
  // Extract the privileged sub graph from the full graph.
  LocEvaluator::Ptr evaluator(new LocEvaluator());
  evaluator->setGraph(navigator.graph().get());
  auto privileged_path = navigator.graph()->getSubgraph(0ul, evaluator);
  for (auto it = privileged_path->begin(0ul); it != privileged_path->end();
       ++it) {
    LOG(INFO) << it->v()->id();
    sequence.push_back(it->v()->id());
  }

  navigator.tactic()->setPath(sequence);
  navigator.tactic()->publishPath(node->now());
  navigator.tactic()->setPipeline(PipelineMode::Following);
  navigator.tactic()->addRun();

  // Wait for shutdown
  rclcpp::spin(node);
  rclcpp::shutdown();
}