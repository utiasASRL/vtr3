#include <filesystem>
#include <vtr_common/utils/filesystem.hpp>
#include <vtr_logging/logging_init.hpp>
#include <vtr_tactic/types.hpp>  // TemporalEvaluator
#include "rclcpp/rclcpp.hpp"

namespace fs = std::filesystem;

using namespace vtr;

int main(int argc, char** argv) {
  fs::path data_dir{fs::current_path()};
  if (argc > 1)
    common::utils::expand_user(common::utils::expand_env(data_dir = argv[1]));

  auto graph = pose_graph::RCGraph::LoadOrCreate(data_dir / "graph.index", 0);

  LOG(WARNING) << "[Navigator] Loaded pose graph has " << graph->numberOfRuns()
               << " runs and " << graph->numberOfVertices()
               << " vertices in total.";
  if (!graph->numberOfVertices()) return 0;

  /// Create a temporal evaluator
  tactic::TemporalEvaluator::Ptr evaluator(new tactic::TemporalEvaluator());
  evaluator->setGraph(graph.get());

  /// Iterate over all runs
  double total_length = 0;
  for (auto iter = graph->runs().begin(); iter != graph->runs().end(); iter++) {
    if (iter->second->vertices().empty()) continue;
    auto graph_run =
        graph->getSubgraph(tactic::VertexId(iter->first, 0), evaluator);
    tactic::VertexId::Vector sequence;
    for (auto it = graph_run->begin(tactic::VertexId(iter->first, 0));
         it != graph_run->end(); ++it) {
      // LOG(INFO) << it->v()->id();
      sequence.push_back(it->v()->id());
    }

    tactic::LocalizationChain chain(graph);
    chain.setSequence(sequence);
    chain.expand();
    LOG(INFO) << "Length of the run " << iter->first
              << " is: " << chain.length() << " m.";
    total_length += chain.length();
  }
  LOG(INFO) << "Total length of this graph is: " << total_length << " m.";
}