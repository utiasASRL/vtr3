#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_messages/msg/run_to_cosine_distance.hpp>
#include <vtr_tactic/modules/stereo/localization/experience_triage_module.hpp>
#include <vtr_tactic/modules/stereo/localization/tod_recognition_module.hpp>
#include <vtr_vision/messages/bridge.hpp>

namespace vtr {
namespace tactic {

void TodRecognitionModule::runImpl(QueryCache &qdata, MapCache &,
                                   const Graph::ConstPtr &graph) {
  // Initialize some basic variables
  VertexId live_id = *qdata.live_id;
  Vertex live_vtx = *graph->at(live_id);
  // The experiences that have been recommended so far
  if (!qdata.recommended_experiences) qdata.recommended_experiences.fallback();
  RunIdSet &recommended = *qdata.recommended_experiences;

  // Clear any past status message
  status_msg_ = vtr_messages::msg::ExpRecogStatus();

  // We only do work if there are more runs needed to be recommended
  if ((int)recommended.size() >= config_->num_exp) return;

  // Start the timer
  common::timing::SimpleTimer timer;

  // Get the vertices we'll localize against
  pose_graph::RCGraphBase &submap = **qdata.localization_map;

  // Get the time of day
  time_point time_of_day = common::timing::toChrono(live_vtx.keyFrameTime());

  // Calculate the temporal difference to map times, score by increasing
  // distance
  ScoredRids scored_rids = scoreExperiences(time_of_day, submap, *config_);

  // Figure out the recommended experiences from the sorted runs,
  // recommend them in the cache if this module is in the loop
  RunIdSet newly_recommended =
      fillRecommends(config_->in_the_loop ? &recommended : nullptr, scored_rids,
                     config_->num_exp);

  // Done! write up the status message
  // ---------------------------------
  auto run_time = timer.elapsedMs();
  //  recordResults(graph->at(status_.live_id), timer.elapsedMs(),
  //                differences, best_exp, in_the_loop);

  // The keyframe time
  status_msg_.keyframe_time = live_vtx.keyFrameTime();
  // The query id
  status_msg_.set__query_id(live_id);
  // The algorithm
  status_msg_.set__algorithm(vtr_messages::msg::ExpRecogStatus::ALGORITHM_TIME);
  // Whether we're running online
  status_msg_.set__in_the_loop(config_->in_the_loop);
  // The bag-of-words cosine distances for each run
  for (const ScoredRid &dist_rid : scored_rids) {
    vtr_messages::msg::RunToCosineDistance dist_msg;
    dist_msg.set__run_id(dist_rid.second);
    dist_msg.set__cosine_distance(dist_rid.first);
    status_msg_.cosine_distances.push_back(dist_msg);
  }
  // The recommended runs for localization
  for (uint32_t rid : newly_recommended)
    status_msg_.recommended_ids.push_back(rid);
  // The computation time
  status_msg_.set__computation_time_ms(run_time);

  // Status message
  LOG_IF(config_->verbose, INFO) << "TOD: " << status_msg_;
}

ScoredRids scoreExperiences(const TodRecognitionModule::time_point &query_tp,
                            const pose_graph::RCGraphBase &submap,
                            const TodRecognitionModule::Config &config) {
  // Conversion from time to time-of-day
  typedef TodRecognitionModule::time_point time_point;
  typedef date::time_of_day<time_point::duration> tod_duration;
  auto time2tod = [](const time_point &tp) -> tod_duration {
    auto day = date::floor<date::days>(tp);
    return date::make_time(tp - time_point(day));
  };

  // Initialize some basic variables
  const tod_duration query_tod = time2tod(query_tp);
  ExperienceDifferences rid_dist;

  // Go through each vertex in the submap
  for (const auto &it : submap) {
    // Get info about the run, make sure we didn't already process it
    const Vertex::Ptr &v = it.v();
    RunId rid = v->id().majorId();
    if (rid_dist.count(rid)) continue;

    // Get the map time point and time of day
    TodRecognitionModule::time_point map_tp =
        common::timing::toChrono(v->keyFrameTime());
    tod_duration map_tod = time2tod(map_tp);

    // Get time and time-of-day difference
    typedef std::chrono::duration<float, std::chrono::hours::period> f_hours;
    auto total_duration =
        query_tp > map_tp ? (query_tp - map_tp) : (map_tp - query_tp);
    float total_durationf = f_hours(total_duration).count();
    auto tod_difference = query_tod.to_duration() - map_tod.to_duration();
    float tod_differencef =
        fabs(fmod(f_hours(tod_difference).count() + 36.f, 24.f) - 12.f);

    // The total time and time-of-day costs based on configured weights
    float total_time_cost = total_durationf * config.total_time_weight;
    float tod_cost = tod_differencef * config.time_of_day_weight;

    // record the difference in the map
    rid_dist[rid] = total_time_cost + tod_cost;
  }

  // Invert the map, sorting the runs from lowest difference to highest
  ScoredRids dist_rids;
  for (const ExperienceDifference &run_diff : rid_dist)
    dist_rids.emplace(run_diff.second, run_diff.first);  // multimap
  return dist_rids;
}

void TodRecognitionModule::updateGraphImpl(QueryCache &, MapCache &,
                                           const Graph::Ptr &graph,
                                           VertexId vid) {
  const Vertex::Ptr &vertex = graph->at(vid);

  // Save the status/results message
  if (status_msg_.query_id == vid) {
    RunId rid = vertex->id().majorId();
    std::string results_stream = "time_of_day_picker";
    graph->registerVertexStream<vtr_messages::msg::ExpRecogStatus>(
        rid, results_stream);
    vertex->insert(results_stream, status_msg_, vertex->keyFrameTime());
  }
}

}  // namespace tactic
}  // namespace vtr
