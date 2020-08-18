#include <algorithm>

#include <vtr/navigation/modules/localization/experience_triage.h>

#include <asrl/messages/LocalizationStatus.pb.h>

namespace std {
std::ostream& operator<<(std::ostream& os,
                         const asrl::status_msgs::ExpRecogStatus& msg) {
  std::ios fmt(nullptr);
  fmt.copyfmt(os);
  os << std::fixed;
  //    if(msg.has_algorithm()) os << msg.Algorithm_Name(msg.algorithm()) << "
  //    ";
  if (msg.has_in_the_loop()) os << (msg.in_the_loop() ? "[itl] " : "[off] ");
  if (msg.has_computation_time_ms())
    os << "run: " << std::setw(5) << std::setprecision(1)
       << msg.computation_time_ms() << " ms ";
  if (msg.has_load_time_ms())
    os << "load: " << std::setw(4) << std::setprecision(1) << msg.load_time_ms()
       << " ms ";
  // If it has distances, this is preferred over straight up recommendations
  if (msg.cosine_distances_size()) {
    os << "nrec: " << msg.recommended_ids_size() << " ";
    std::ostringstream oss;
    oss << std::setprecision(3) << std::fixed;
    for (const auto& run_dist : msg.cosine_distances()) {
      if (oss.str().size() >= 150) {
        oss << "...";
        break;
      }
      oss << std::setw(3) << run_dist.run_id() << ": "
          << run_dist.cosine_distance() << " ";
    }
    os << "dist: " << oss.str();
    // Some only have recommendations, not cosine distances
  } else if (msg.recommended_ids_size()) {
    os << "rec: ";
    for (const auto& rec : msg.recommended_ids())
      os << std::setw(3) << rec << " ";
  }

  os.copyfmt(fmt);
  return os;
}
}  // namespace std

namespace vtr {
namespace navigation {

RunIdSet getRunIds(const asrl::pose_graph::RCGraphBase& graph) {
  RunIdSet rids;
  // iterate through each vertex in the graph
  for (const auto& itr : graph)
    // insert the run id for this vertex in the set
    rids.insert(itr.v()->id().majorId());
  return rids;
}

RunIdSet privelegedRuns(const asrl::pose_graph::RCGraphBase& graph, RunIdSet rids) {
  RunIdSet prids;
  // go through each run in the run id set
  for (RunId rid : rids)
    // check to see if it was a manual (privileged) run
    if (graph.run(rid)->isManual())
      // if it was, insert it in the new privileged set
      prids.insert(rid);
  return prids;
}

asrl::pose_graph::RCGraphBase::Ptr maskSubgraph(
    const asrl::pose_graph::RCGraphBase::Ptr& graph, const RunIdSet& mask) {
  VertexId::Vector kept_vertex_ids;
  kept_vertex_ids.reserve(graph->numberOfVertices());
  // Check all the runs for inclusion in the new masked subgraph
  for (VertexId vid : graph->subgraph().getNodeIds())
    if (mask.count(vid.majorId())) kept_vertex_ids.push_back(vid);
  return graph->getSubgraph(kept_vertex_ids);
}

RunIdSet fillRecommends(RunIdSet* recommends, const ScoredRids& distance_rids,
                        unsigned n) {
  RunIdSet new_recs;
  // go through the scored runs, from highest to lowest
  for (const auto& scored_rid : distance_rids) {
    // if we are supplementing existing recommendations
    if (recommends) {
      // insert the new run until the recommendation is large enough
      if (recommends->size() >= n) break;
      recommends->insert(scored_rid.second);
    }
    // recored the newly recommended runs until ther are enough
    if (new_recs.size() >= n) break;
    new_recs.insert(scored_rid.second);
  }
  // return the newly recommended runs
  return new_recs;
}

void ExperienceTriageModule::run(QueryCache& qdata, MapCache& mdata,
                                 const std::shared_ptr<const Graph>& graph) {
  // Grab what has been recommended so far by upstream recommenders
  RunIdSet& recommended = *mdata.recommended_experiences.fallback();

  // Check if we need to do things...
  // --------------------------------

  asrl::pose_graph::RCGraphBase::Ptr& submap_ptr = *mdata.localization_map;
  if (config_.in_the_loop) {
    // If the mask is empty, we default to using all runs
    if (recommended.empty()) {
      recommended = getRunIds(*submap_ptr);
    } else {
      // If we always want to include the priveleged, make sure they're included
      if (config_.always_privileged) {
        RunIdSet priv_runs = privelegedRuns(*graph, getRunIds(*submap_ptr));
        recommended.insert(priv_runs.begin(), priv_runs.end());
      }

      // Apply the mask to the localization subgraph
      submap_ptr = maskSubgraph(submap_ptr, recommended);
      if (mdata.localization_status)
        mdata.localization_status->set_window_num_vertices(
            submap_ptr->numberOfVertices());
    }
  }

  // Build the status message we'll save out to the graph
  // ----------------------------------------------------

  // Basic status info
  Vertex::Ptr query_vertex = graph->at(*qdata.live_id);
  status_msg_.Clear();
  status_msg_.set_in_the_loop(config_.in_the_loop);
  status_msg_.mutable_keyframe_time()->CopyFrom(query_vertex->keyFrameTime());
  status_msg_.set_query_id(query_vertex->id());

  // The recommended runs for localization
  for (const RunId& rid : recommended) status_msg_.add_recommended_ids(rid);

  LOG_IF(config_.verbose, INFO) << "ET: " << status_msg_;
}

void ExperienceTriageModule::updateGraph(QueryCache& qdata, MapCache& mdata,
                                         const std::shared_ptr<Graph>& graph,
                                         VertexId live_id) {
  // Save the status/results message
  // -------------------------------

  // Make sure the saved status message matches the live id
  if (status_msg_.query_id() == live_id) {
    Vertex::Ptr vertex = graph->at(live_id);
    RunId rid = vertex->id().majorId();
    std::string results_stream = "/results/experience_triage";
    if (!graph->hasVertexStream(rid, results_stream))
      graph->registerVertexStream(rid, results_stream, true);
    vertex->insert(results_stream, status_msg_, vertex->keyFrameTime());
  }
}

}  // namespace navigation
}  // namespace vtr
