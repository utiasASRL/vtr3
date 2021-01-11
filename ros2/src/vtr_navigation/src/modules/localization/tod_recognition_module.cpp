#if false
#include <asrl/navigation/modules/localization/ExperienceTriage.hpp>
#include <asrl/navigation/modules/matching/TodRecognitionModule.hpp>
#include <asrl/common/timing/SimpleTimer.hpp>
#include <asrl/vision/messages/bridge.hpp>

namespace asrl {
namespace navigation {

void TodRecognitionModule::run(QueryCache & qdata, MapCache & mdata,
                               const std::shared_ptr<const Graph> & graph) {

  // Initialize some basic variables
  VertexId live_id = *qdata.live_id; // The live id we're trying to localize
  Vertex live_vtx = *graph->at(live_id); // The live vertex we're trying to localize
  // The experiences that have been recommended so far
  RunIdSet & recommended = *mdata.recommended_experiences.fallback();
  status_msg_.Clear(); // Clear any past status messages

  // We only do work if there are more runs needed to be recommended
  if ((int)recommended.size() >= config_.num_exp) return;

  // Start the timer
  common::timing::SimpleTimer timer;

  // Get the vertices we'll localize against
  pose_graph::RCGraphBase & submap = **mdata.localization_map;

  // Get the time of day
  time_point time_of_day = common::timing::toChrono(live_vtx.keyFrameTime());

  // Calculate the temporal difference to map times, score by increasing distance
  ScoredRids scored_rids = scoreExperiences(time_of_day, submap, config_);

  // Figure out the recommended experiences from the sorted runs,
  // recommend them in the cache if this module is in the loop
  RunIdSet newly_recommended = fillRecommends(
        config_.in_the_loop ? &recommended : nullptr, scored_rids, config_.num_exp);

  // Done! write up the status message
  // ---------------------------------
  auto run_time = timer.elapsedMs();
//  recordResults(graph->at(status_.live_id), timer.elapsedMs(),
//                differences, best_exp, in_the_loop);

  // The keyframe time
  status_msg_.mutable_keyframe_time()->CopyFrom(live_vtx.keyFrameTime());
  // The query id
  status_msg_.set_query_id(live_id);
  // The algorithm
  status_msg_.set_algorithm(status_msgs::ExpRecogStatus_Algorithm_TIME);
  // Whether we're running online
  status_msg_.set_in_the_loop(config_.in_the_loop);
  // The bag-of-words cosine distances for each run
  for (const ScoredRid & dist_rid : scored_rids) {
    auto & dist_msg = *status_msg_.add_cosine_distances();
    dist_msg.set_run_id(dist_rid.second);
    dist_msg.set_cosine_distance(dist_rid.first);
  }
  // The recommended runs for localization
  for (uint32_t rid : newly_recommended)
    status_msg_.add_recommended_ids(rid);
  // The computation time
  status_msg_.set_computation_time_ms(run_time);

  // Status message
  LOG_IF(config_.verbose, INFO) << "TOD: " << status_msg_;
}

ScoredRids
scoreExperiences(
    const TodRecognitionModule::time_point & query_tp,
    const pose_graph::RCGraphBase & submap,
    const TodRecognitionModule::Config & config) {

  // Conversion from time to time-of-day
  typedef TodRecognitionModule::time_point time_point;
  typedef date::time_of_day<time_point::duration> tod_duration;
  auto time2tod = [](const time_point & tp) -> tod_duration {
    auto day = date::floor<date::days>(tp);
    return date::make_time(tp - time_point(day));
  };

  // Initialize some basic variables
  const tod_duration query_tod = time2tod(query_tp);
  ExperienceDifferences rid_dist;

  // Go through each vertex in the submap
  for (const auto & it : submap) {
    // Get info about the run, make sure we didn't already process it
    const Vertex::Ptr & v = it.v();
    RunId rid = v->id().majorId();
    if (rid_dist.count(rid)) continue;

    // Get the map time point and time of day
    TodRecognitionModule::time_point map_tp = common::timing::toChrono(v->keyFrameTime());
    tod_duration map_tod = time2tod(map_tp);

    // Get time and time-of-day difference
    typedef std::chrono::duration<float,std::chrono::hours::period> f_hours;
    auto total_duration = query_tp > map_tp ? (query_tp-map_tp) : (map_tp-query_tp);
    float total_durationf = f_hours(total_duration).count();
    auto tod_difference = query_tod.to_duration() - map_tod.to_duration();
    float tod_differencef = fabs(fmod(f_hours(tod_difference).count()+36.f, 24.f)-12.f);

    // The total time and time-of-day costs based on configured weights
    float total_time_cost = total_durationf * config.total_time_weight;
    float tod_cost = tod_differencef * config.time_of_day_weight;

    // record the difference in the map
    rid_dist[rid] = total_time_cost + tod_cost;
  }

  // Invert the map, sorting the runs from lowest difference to highest
  ScoredRids dist_rids;
  for (const ExperienceDifference & run_diff : rid_dist)
    dist_rids.emplace(run_diff.second, run_diff.first); // multimap
  return dist_rids;
}

void TodRecognitionModule::updateGraph(QueryCache & qdata, MapCache &,
                                       const std::shared_ptr<Graph> &graph,
                                       VertexId vid) {
  const Vertex::Ptr & vertex = graph->at(vid);

  // Save the status/results message
  if (status_msg_.query_id() == vid) {
    RunId rid = vertex->id().majorId();
    std::string results_stream = "/results/time_of_day_picker";
    if (!graph->hasVertexStream(rid, results_stream)) {
      graph->registerVertexStream(rid, results_stream, true);
    }
    vertex->insert(results_stream, status_msg_, vertex->keyFrameTime());
  }
}

}
}
#endif
