#include <random>
#include <vtr_navigation/modules/localization/random_experiences.hpp>
#include <vtr_navigation/modules.hpp>

namespace vtr {
namespace navigation {

void RandomExperiencesModule::run(QueryCache &qdata, MapCache &mdata,
                                  const std::shared_ptr<const Graph> &graph) {

  // Initialize some basic variables
  VertexId live_id = *qdata.live_id; // The live id we're trying to localize
  Vertex live_vtx = *graph->at(live_id); // The live vertex we're trying to localize
  // The experiences that have been recommended so far
  RunIdSet &recommended = *mdata.recommended_experiences.fallback();
  status_msg_ = vtr_messages::msg::ExpRecogStatus(); // Clear any past status messages

  // We only do work if there are more runs needed to be recommended
  if ((int) recommended.size() >= config_.num_exp) return;

  // Start the timer
  common::timing::SimpleTimer timer;

  // Get the vertices we'll localize against
  pose_graph::RCGraphBase &submap = **mdata.localization_map;

  // Get the runs to choose from
  RunIdSet run_ids = getRunIds(submap);

  // Calculate a random score from 0 to 1 for each of the runs
  std::mt19937 gen(live_vtx.keyFrameTime().nanoseconds_since_epoch);
  std::uniform_real_distribution<float> dist;
  ScoredRids scored_rids;
  for (const RunId &rid : run_ids) scored_rids.emplace(dist(gen), rid);

  // Figure out the recommended experiences from the sorted runs,
  // recommend them in the cache if this module is in the loop
  RunIdSet newly_recommended = fillRecommends(
      config_.in_the_loop ? &recommended : nullptr, scored_rids, config_.num_exp);

  // Done! write up the status message
  // ---------------------------------
  auto run_time = timer.elapsedMs();

  // The keyframe time
  status_msg_.keyframe_time = live_vtx.keyFrameTime();
  // The query id
  status_msg_.set__query_id(live_id);
  // The algorithm
  status_msg_.set__algorithm(vtr_messages::msg::ExpRecogStatus::ALGORITHM_RANDOM);
  // Whether we're running online
  status_msg_.set__in_the_loop(config_.in_the_loop);
  // The recommended runs for localization
  for (uint32_t rid : newly_recommended)
    status_msg_.recommended_ids.push_back(rid);
  // The computation time
  status_msg_.set__computation_time_ms(run_time);

  // Status message
  LOG_IF(config_.verbose, INFO) << "RND: " << status_msg_;
}

void RandomExperiencesModule::updateGraph(QueryCache &qdata, MapCache &,
                                          const std::shared_ptr<Graph> &graph,
                                          VertexId vid) {
  const Vertex::Ptr &vertex = graph->at(vid);

  // Save the status/results message
  if (status_msg_.query_id == vid) {
    RunId rid = vertex->id().majorId();
    std::string results_stream = "random_experiences";
    if (!graph->hasVertexStream(rid, results_stream)) {
      graph->registerVertexStream<vtr_messages::msg::ExpRecogStatus>(rid, results_stream, true);
    }
    vertex->insert(results_stream, status_msg_, vertex->keyFrameTime());
  }
}

}
}
