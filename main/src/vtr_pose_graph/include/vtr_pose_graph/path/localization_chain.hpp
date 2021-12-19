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
 * \file localization_chain.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 * \brief LocalizationChain class definition
 * \details
 * A description of the localization chain
 * ---------------------------------------
 *
 * Live   |> === |> === |> === Tw ==|>==|>==Pe .. Le
 *               |      |       x                *
 *               |      |       x              *
 * Priv.  |> === |> === |> === Br ==|>==|>=== Tr
 *
 * (Le) Leaf    --- The latest frame that's currently being processed
 * (Pe) Petiole --- The latest live vertex.
 * (Tr) Trunk   --- The privileged vertex that is estimated closest to the Leaf
 * (Tw) Twig    --- The live vertex that is part of the current localization
 *                  problem.
 * (Br) Branch  --- The privileged vertex that is part of the current
 *                  localization problem.
 *
 * ===          --- Temporal edge, these vertices are adjacent in time
 * |            --- Spatial edge, these vertices have been localized to each
 *                  other
 * xx           --- T_Br_Tr, the cumulative transform using the last
 *                  localization
 * **           --- T_Le_Tr, T_Le_Br * T_Br_Tr (cumulative plus VO as a prior)
 *
 */
#pragma once

#include <algorithm>

#include "vtr_pose_graph/path/path.hpp"
#include "vtr_pose_graph/serializable/rc_graph.hpp"

namespace vtr {
namespace pose_graph {

template <class Graph>
class LocalizationChain : public Path<Graph> {
 public:
  struct Config {
    double min_cusp_distance = 1.5;
    double angle_weight = 7.0;
    int search_depth = 20;
    int search_back_depth = 10;
    double distance_warning = 3.0;
  };

  using Parent = Path<Graph>;
  using Sequence = typename Parent::Sequence;

  // Basic graph typedefs
  using Edge = typename Graph::EdgeType;
  using Vertex = typename Graph::VertexType;

  // Often used typedefs
  using VertexId = typename Vertex::IdType;
  using TF = typename Edge::TransformType;

  // thread safety
  // using Parent::mutex_;
  using typename Parent::LockGuard;
  using typename Parent::Mutex;
  using typename Parent::UniqueLock;

  PTR_TYPEDEFS(LocalizationChain);

  LocalizationChain(const Config &config, const typename Graph::Ptr &graph)
      : Parent(graph), config_(config) {}

  LocalizationChain(const typename Graph::Ptr &graph)
      : LocalizationChain(Config(), graph) {}

  bool isLocalized() const {
    LockGuard lock(this->mutex_);
    return is_localized_;
  }

  unsigned trunkSequenceId() const {
    LockGuard lock(this->mutex_);
    return trunk_sid_;
  }
  unsigned branchSequenceId() const {
    LockGuard lock(this->mutex_);
    return branch_sid_;
  }

  VertexId petioleVertexId() const {
    LockGuard lock(this->mutex_);
    return petiole_vid_;
  }
  VertexId twigVertexId() const {
    LockGuard lock(this->mutex_);
    return twig_vid_;
  }
  VertexId branchVertexId() const {
    LockGuard lock(this->mutex_);
    return branch_vid_;
  }
  VertexId trunkVertexId() const {
    LockGuard lock(this->mutex_);
    return trunk_vid_;
  }

  TF T_leaf_petiole() const {
    LockGuard lock(this->mutex_);
    return T_leaf_petiole_;
  }
  TF T_leaf_twig() const {
    LockGuard lock(this->mutex_);
    return T_leaf_petiole_ * T_petiole_twig_;
  }
  TF T_leaf_trunk() const {
    LockGuard lock(this->mutex_);
    return T_leaf_petiole_ * T_petiole_twig_ * T_twig_branch_ * T_branch_trunk_;
  }
  TF T_petiole_twig() const {
    LockGuard lock(this->mutex_);
    return T_petiole_twig_;
  }
  TF T_petiole_trunk() const {
    LockGuard lock(this->mutex_);
    return T_petiole_twig_ * T_twig_branch_ * T_branch_trunk_;
  }
  TF T_twig_branch() const {
    LockGuard lock(this->mutex_);
    return T_twig_branch_;
  }
  TF T_twig_trunk() const {
    LockGuard lock(this->mutex_);
    return T_twig_branch_ * T_branch_trunk_;
  }
  TF T_branch_trunk() const {
    LockGuard lock(this->mutex_);
    return T_branch_trunk_;
  }

  /// What is the privileged vehicle pose (relative to the start of the path)
  TF T_start_twig() {
    LockGuard lock(this->mutex_);
    return this->pose(trunk_sid_) * T_twig_trunk().inverse();
  }
  TF T_start_petiole() {
    LockGuard lock(this->mutex_);
    return this->pose(trunk_sid_) * T_petiole_trunk().inverse();
  }
  TF T_start_leaf() {
    LockGuard lock(this->mutex_);
    return this->pose(trunk_sid_) * T_leaf_trunk().inverse();
  }
  TF T_start_trunk() {
    LockGuard lock(this->mutex_);
    return this->pose(trunk_sid_);
  }

  TF T_trunk_target(unsigned seq_id) const;

  /** \brief Resets localization chain to its initial state. */
  void reset();

  /**
   * \brief Resets the vertex we think we're the closest to and unset
   * is_localized status.
   */
  void resetTrunk(unsigned trunk_sid);

  /** \brief Updates T_leaf_twig from odometry */
  void updatePetioleToLeafTransform(const TF &T_leaf_twig,
                                    const bool search_closest_trunk,
                                    const bool look_backwards = false);

  /** \brief Updates Petiole and reset leaf petiole transform */
  void setPetiole(const VertexId &petiole_id);

  /**
   * \brief Move the localization chain forward.
   * \details
   *  Tw-->-->--Pe-->Le            >-->-->--Tw,Pe-->Le
   *  |                    ===>    |
   *  Br-->-->-->-->--Tr           >-->-->-->-->--Br,Tr-->
   */
  void convertPetioleTrunkToTwigBranch();

  void initializeBranchToTwigTransform(const TF &T_twig_branch);

  void updateBranchToTwigTransform(const VertexId &twig_vid,
                                   const VertexId &branch_vid,
                                   const unsigned &branch_sid,
                                   const TF &T_twig_branch,
                                   const bool search_closest_trunk,
                                   const bool search_backwards = false);

 protected:
  /** \brief Initializes privileged path to localize against. */
  void initSequence();

  /** \brief Update the Trunk to the closest vertex. */
  void searchClosestTrunk(bool look_backwards);

  /** \brief important indices */
  unsigned trunk_sid_ = (unsigned)-1;
  unsigned branch_sid_ = (unsigned)-1;

  /** \brief important vertices (see description at the top) */
  VertexId petiole_vid_ = VertexId::Invalid();
  VertexId twig_vid_ = VertexId::Invalid();
  VertexId branch_vid_ = VertexId::Invalid();
  VertexId trunk_vid_ = VertexId::Invalid();

  /** \brief important transforms (default to identity with zero cov) */
  TF T_leaf_petiole_ = TF(true);  // frame-to-kf
  TF T_petiole_twig_ = TF(true);  // Autonomous edges
  TF T_twig_branch_ = TF(true);   // Localization
  TF T_branch_trunk_ = TF(true);  // Privileged edges

  /** \brief localization status */
  bool is_localized_ = false;

  /** \brief configuration */
  const Config config_;
};

extern template class LocalizationChain<pose_graph::BasicGraph>;
extern template class LocalizationChain<pose_graph::RCGraph>;

}  // namespace pose_graph
}  // namespace vtr

template <class Graph>
std::ostream &operator<<(
    std::ostream &stream,
    const vtr::pose_graph::LocalizationChain<Graph> &chain);
