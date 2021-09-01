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
 * \file pose_cache.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/evaluator/accumulators.hpp>
#include <vtr_pose_graph/evaluator/mask_evaluator.hpp>
#include <vtr_pose_graph/index/graph.hpp>

namespace vtr {
namespace pose_graph {

namespace {
namespace Mask = eval::Mask;
namespace Weight = eval::Weight;
}  // namespace

/**
 * \class PoseCache
 * \brief caches the poses between a root vertex and query vertices given a
 * graph
 */
template <class G, class TF = typename G::EdgeType::TransformType>
class PoseCache {
 public:
  // Graph typedefs
  using GraphType = G;
  using GraphConstPtr = typename G::ConstPtr;
  using VertexIdType = typename G::VertexIdType;
  using SequenceType = typename VertexIdType::Vector;
  using tf_t = TF;

  /**
   * \brief constructor
   * \param graph A pointer to the graph to search over.
   * \param root_id The root vertex that all the poses are wrt.
   */
  PoseCache(const GraphConstPtr& graph, VertexIdType root_id)
      : graph_(graph), root_id_(root_id) {
    tf_map_[root_id_] = tf_t(true);
  }

  /** \brief default destructor */
  ~PoseCache() = default;

  /**
   * \brief Get the TF that takes points from the query to the root pose.
   * \param query_id The id of the query vertex.
   * \param mask Optional mask of vertices.
   */
  tf_t T_root_query(VertexId query_id,
                    const Mask::Ptr& mask = Mask::Const::MakeShared(true,
                                                                    true)) {
    // check to see if we have calculated this transform already
    // this pre-check avoids having to do the breadth first search
    auto query_it = tf_map_.find(query_id);
    if (query_it != tf_map_.end()) return query_it->second;

    // use bfs to get the path between root and query
    // TODO use a search starting at query_id to any cached id...
    auto path = graph_->dijkstraSearch(
        root_id_, query_id, Weight::Const::MakeShared(1.f, 1.f), mask);

    // Start with itr->to() being the vertex id one past root_id_
    auto itr = ++path->begin(root_id_);
    // Pointer to the cached transform representing the iterator (updated in
    // loop)
    tf_t* T_root_curr_ptr = &tf_map_.at(root_id_);

    // Work from root toward the query, (caching all the way, hahaha)
    for (; itr != path->end(); ++itr) {
      // Look for the current id in the cache
      auto T_cached_it = tf_map_.find(itr->to());
      // Update the cache if we didn't find it
      if (T_cached_it == tf_map_.end()) {
        // T_root_curr = T_root_prev * T_prev_curr
        auto&& T_root_curr = (*T_root_curr_ptr) * itr->T();
        T_cached_it = tf_map_.emplace(itr->to(), T_root_curr).first;
      }
      // Remember the cached value
      T_root_curr_ptr = &T_cached_it->second;
    }

    // return the cached TF.
    return *T_root_curr_ptr;
  }

 private:
  /** \brief a pointer to the graph. */
  GraphConstPtr graph_;

  /** \brief The id of the root vertex. */
  VertexIdType root_id_;

  /** \brief The cached transforms */
  std::map<VertexIdType, tf_t> tf_map_;
};

}  // namespace pose_graph
}  // namespace vtr
