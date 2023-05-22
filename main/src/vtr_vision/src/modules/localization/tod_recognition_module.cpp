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
 * \file tod_recognition_module.cpp
 * \brief TodRecognitionModule class method definitions
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_common/timing/stopwatch.hpp>
#include <vtr_messages/msg/run_to_cosine_distance.hpp>
#include <vtr_vision/messages/bridge.hpp>
#include <vtr_vision/modules/localization/tod_recognition_module.hpp>
#include <vtr_vision/utils.hpp>

namespace vtr {
namespace vision {

using namespace tactic;

auto TodRecognitionModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix) 
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->verbose = node->declare_parameter<bool>(param_prefix + ".verbose", config->verbose);
  config->num_exp = node->declare_parameter<int>(param_prefix + ".num_desired_experiences", config->num_exp);
  config->in_the_loop = node->declare_parameter<bool>(param_prefix + ".in_the_loop", config->in_the_loop);
  config->time_of_day_weight = node->declare_parameter<float>(param_prefix + ".time_of_day_weight", config->time_of_day_weight);
  config->total_time_weight = node->declare_parameter<float>(param_prefix + ".total_time_weight", config->total_time_weight);
  // clang-format on

  return config;
}

void TodRecognitionModule::run_(tactic::QueryCache &qdata0, tactic::OutputCache &output, const tactic::Graph::Ptr &graph,
                const std::shared_ptr<tactic::TaskExecutor> &executor) {
  auto &qdata = dynamic_cast<CameraQueryCache &>(qdata0);

  // Initialize some basic variables
  VertexId live_id = *qdata.vid_odo;
  VertexPtr live_vtx = graph->at(live_id);
  // The experiences that have been recommended so far
  if (!qdata.recommended_experiences) qdata.recommended_experiences.emplace();
  RunIdSet &recommended = *qdata.recommended_experiences;

  // Clear any past status message
  auto status_msg = vtr_messages::msg::ExpRecogStatus();

  // We only do work if there are more runs needed to be recommended
  if ((int)recommended.size() >= config_->num_exp) return;

  // Start the timer
  common::timing::Stopwatch timer;


  // Get the vertices we'll localize against
  auto &submap = *qdata.localization_map;

  // Get the time of day
  time_point time_of_day = common::timing::toChrono(live_vtx->vertexTime());

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
  timer.stop();
  auto run_time = timer.count();
  //  recordResults(graph->at(status_.live_id), timer.elapsedMs(),
  //                differences, best_exp, in_the_loop);

  // The keyframe time
  status_msg.keyframe_time.nanoseconds_since_epoch = live_vtx->vertexTime();
  // The query id
  status_msg.set__query_id(live_id);
  // The algorithm
  status_msg.set__algorithm(vtr_messages::msg::ExpRecogStatus::ALGORITHM_TIME);
  // Whether we're running online
  status_msg.set__in_the_loop(config_->in_the_loop);
  // The bag-of-words cosine distances for each run
  for (const ScoredRid dist_rid : scored_rids) {
    vtr_messages::msg::RunToCosineDistance dist_msg;
    dist_msg.set__run_id(dist_rid.second);
    dist_msg.set__cosine_distance(dist_rid.first);
    status_msg.cosine_distances.push_back(dist_msg);
  }
  // The recommended runs for localization
  for (uint32_t rid : newly_recommended)
    status_msg.recommended_ids.push_back(rid);
  // The computation time
  status_msg.set__computation_time_ms(run_time);

  // Status message
  CLOG_IF(config_->verbose, DEBUG, "stereo.tod") << "TOD Recommendations: " << status_msg.recommended_ids;
}

ScoredRids scoreExperiences(const TodRecognitionModule::time_point &query_tp,
                            const tactic::GraphBase::Ptr &submap,
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
  for (const auto &it : *submap) {
    // Get info about the run, make sure we didn't already process it
    const Vertex::Ptr &v = it.v();
    RunId rid = v->id().majorId();
    if (rid_dist.count(rid)) continue;

    // Get the map time point and time of day
    TodRecognitionModule::time_point map_tp =
        common::timing::toChrono(v->vertexTime());
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
  for (const ExperienceDifference run_diff : rid_dist)
    dist_rids.emplace(run_diff.second, run_diff.first);  // multimap
  return dist_rids;
}

void TodRecognitionModule::storeRunSelection(QueryCache &,
                                           const Graph::Ptr &graph,
                                           VertexId vid,
                                           vtr_messages::msg::ExpRecogStatus status_msg) {
  const Vertex::Ptr &vertex = graph->at(vid);

  // \todo store result in the pose graph.
  // Add param to do so.

  // Save the status/results message
  // if (status_msg.query_id == vid) {
  //   RunId rid = vertex->id().majorId();
  //   std::string results_stream = "time_of_day_picker";
  //   graph->registerVertexStream<vtr_messages::msg::ExpRecogStatus>(
  //       rid, results_stream);
  //   vertex->insert(results_stream, status_msg, vertex->vertexTime());
  // }
}

}  // namespace vision
}  // namespace vtr