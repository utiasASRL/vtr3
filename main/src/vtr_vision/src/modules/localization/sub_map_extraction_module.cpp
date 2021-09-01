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
 * \file sub_map_extraction_module.cpp
 * \brief SubMapExtractionModule class method definitions
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_messages/msg/localization_status.hpp>
#include <vtr_pose_graph/evaluator/accumulators.hpp>
#include <vtr_pose_graph/path/pose_cache.hpp>
#include <vtr_vision/modules/localization/sub_map_extraction_module.hpp>

namespace vtr {
namespace vision {

using namespace tactic;

void SubMapExtractionModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                                           const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->sigma_scale = node->declare_parameter<double>(param_prefix + ".sigma_scale", config_->sigma_scale);
  config_->temporal_min_depth = node->declare_parameter<int>(param_prefix + ".temporal_min_depth", config_->temporal_min_depth);
  config_->temporal_max_depth = node->declare_parameter<int>(param_prefix + ".temporal_max_depth", config_->temporal_max_depth);
  config_->search_spatially = node->declare_parameter<bool>(param_prefix + ".search_spatially", config_->search_spatially);
  config_->angle_weight = node->declare_parameter<double>(param_prefix + ".angle_weight", config_->angle_weight);
  // clang-format on
}

void SubMapExtractionModule::runImpl(QueryCache &qdata0,
                                     const Graph::ConstPtr &graph) {
  auto &qdata = dynamic_cast<CameraQueryCache &>(qdata0);

  qdata.localization_status.fallback();
  // Grab the id we wish to center the map on.
  auto root = *qdata.map_id;

  // sanity check
  if (root == VertexId::Invalid()) {
    LOG(ERROR) << "Root vertex is invalid. Not extracting submap.";
    return;
  }

  // Save off the id of the current run.
  auto current_run = (*qdata.live_id).majorId();

  EdgeTransform &T_q_m = *qdata.T_r_m_prior;
  auto lateral_uncertainty = sqrt((T_q_m.cov()(0, 0))) * config_->sigma_scale;

  // Figure out how many vertices of uncertainty there are.
  int depth = std::max(config_->temporal_min_depth,
                       calculateDepth(root, lateral_uncertainty, graph));
  (*qdata.localization_status).window_temporal_depth = depth;
  // If the experiences have been masked, get the subset
  RunIdSet *mask = qdata.recommended_experiences.is_valid()
                       ? &(*qdata.recommended_experiences)
                       : nullptr;

  // Get the subgraph that connects the vertices in the vector.
  auto localization_map = extractSubmap(*graph, root, current_run, mask, depth,
                                        config_->search_spatially);
  qdata.localization_map.fallback(localization_map);
  (*qdata.localization_status).window_num_vertices =
      (*qdata.localization_map)->numberOfVertices();
}

pose_graph::RCGraphBase::Ptr SubMapExtractionModule::extractSubmap(
    const Graph &graph, const VertexId &root, uint32_t current_run,
    RunIdSet *mask, int temporal_depth, bool spatial_neighbours) {
  // Acquire a lock to avoid modification during graph traversal
  const auto lock = graph.guard();

  // Iterate on the temporal edges to get the window.
  PrivilegedEvaluator::Ptr evaluator(new PrivilegedEvaluator());
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
  // Acquire a lock to avoid modification during graph traversal
  const auto lock = graph->guard();

  // Set up the evaluator to only iterate on privileged edges.
  PrivilegedEvaluator::Ptr evaluator(new PrivilegedEvaluator());
  evaluator->setGraph((void *)graph.get());
  EdgeTransform init(true);

  // Start searching starting at the root vertex
  pose_graph::PoseCache<pose_graph::RCGraph> pose_cache(graph, root);
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
          std::make_shared<pose_graph::eval::Weight::Const>(1., 1.), evaluator);
      return to_limit->numberOfVertices() - 1;
    }
  }

  // Otherwise return configured depth
  return config_->temporal_max_depth;
}

}  // namespace vision
}  // namespace vtr