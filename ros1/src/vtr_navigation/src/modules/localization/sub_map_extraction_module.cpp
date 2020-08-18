#include <vtr/navigation/modules/localization/sub_map_extraction_module.h>

#include <asrl/messages/LocalizationStatus.pb.h>
#include <asrl/pose_graph/evaluator/Accumulators.hpp>
#include <asrl/pose_graph/path/PoseCache.hpp>

namespace vtr {
namespace navigation {

void SubMapExtractionModule::run(QueryCache &qdata, MapCache &mdata,
                                 const std::shared_ptr<const Graph> &graph) {
  mdata.localization_status.fallback();
  // Grab the id we wish to center the map on.
  auto root = *mdata.map_id;

  // sanity check
  if (root == VertexId::Invalid()) {
    LOG(ERROR) << "Root vertex is invalid. Not extracting submap.";
    return;
  }

  // Save off the id of the current run.
  auto current_run = (*qdata.live_id).majorId();

  EdgeTransform &T_q_m = *mdata.T_q_m_prior;
  auto lateral_uncertainty = sqrt((T_q_m.cov()(0, 0))) * config_->sigma_scale;

  // Figure out how many vertices of uncertainty there are.
  int depth = std::max(config_->temporal_min_depth,
                       calculateDepth(root, lateral_uncertainty, graph));
  (*mdata.localization_status).set_window_temporal_depth(depth);
  // If the experiences have been masked, get the subset
  RunIdSet *mask = mdata.recommended_experiences.is_valid()
                       ? &(*mdata.recommended_experiences)
                       : nullptr;

  // Get the subgraph that connects the vertices in the vector.
  mdata.localization_map = extractSubmap(*graph, root, current_run, mask, depth,
                                         config_->search_spatially);
  (*mdata.localization_status)
      .set_window_num_vertices((*mdata.localization_map)->numberOfVertices());
}

asrl::pose_graph::RCGraphBase::Ptr SubMapExtractionModule::extractSubmap(
    const Graph &graph, const VertexId &root, uint32_t current_run,
    RunIdSet *mask, int temporal_depth, bool spatial_neighbours) {
  // Iterate on the temporal edges to get the window.
  PrivilegedEvaluatorPtr evaluator(new PrivilegedEvaluator());
  evaluator->setGraph((void *)&graph);

  // Iterate through all of the temporally adjacent vertices in the path
  std::vector<VertexId> vertices;
  auto itr = graph.beginDfs(root, temporal_depth, evaluator);
  for (; itr != graph.end(); ++itr) {
    auto current_vertex = itr->v();

    // add the current, privileged vertex.
    vertices.push_back(current_vertex->id());

    // If spatial search is enabled, then add all spatial neighbors.
    if (spatial_neighbours) {
      // Iterate through spatial neighbors, and add them.
      auto spatial_neighbours = current_vertex->spatialNeighbours();
      for (auto &neighbour : spatial_neighbours) {
        // don't add the live run to the localization map
        if (neighbour.majorId() == current_run) continue;
        // if there is a mask of experiences, check that we're in it
        if (mask && mask->find(neighbour.majorId()) == mask->end()) continue;
        // now that the checks have passed, add it to the list
        vertices.push_back(neighbour);
      }
    }  // end if search_spatially
  }    // end for itr

  return graph.getSubgraph(vertices);
}

int SubMapExtractionModule::calculateDepth(
    VertexId root, double lateral_uncertainty,
    const std::shared_ptr<const Graph> &graph) {
  // Set up the evaluator to only iterate on privileged edges.
  PrivilegedEvaluatorPtr evaluator(new PrivilegedEvaluator());
  evaluator->setGraph((void *)graph.get());
  EdgeTransform init(true);

  // Start searching starting at the root vertex
  asrl::pose_graph::PoseCache<asrl::pose_graph::RCGraph> pose_cache(graph,
                                                                    root);
  auto itr = ++graph->beginBfs(root, config_->temporal_max_depth, evaluator);
  for (; itr != graph->end(); ++itr) {
    auto curr_vid = itr->v()->id();
    // get the transform between this vertex and the root.
    auto T_root_curr = pose_cache.T_root_query(curr_vid, evaluator);
    auto se3_root_curr = T_root_curr.vec();

    // calculate the distance.
    double distance = se3_root_curr.head<3>().norm() +
                      config_->angle_weight * se3_root_curr.tail<3>().norm();

    // If we have exceeded the uncertainty, then calculate and return the depth.
    if (distance >= lateral_uncertainty) {
      auto to_limit = graph->dijkstraSearch(
          root, curr_vid,
          std::make_shared<asrl::pose_graph::Eval::Weight::Const>(1., 1.),
          evaluator);
      return to_limit->numberOfVertices() - 1;
    }
  }

  // Otherwise return configured depth
  return config_->temporal_max_depth;
}

}  // namespace navigation
}  // namespace vtr
