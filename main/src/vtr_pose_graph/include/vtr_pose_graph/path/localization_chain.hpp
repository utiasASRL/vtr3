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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <algorithm>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#include <vtr_pose_graph/path/path.hpp>

namespace vtr {
namespace pose_graph {

class LocalizationChain : public Path<RCGraph> {
 public:
  struct Config {
    double min_cusp_distance = 1.5;
    double angle_weight = 7.0;
    int search_depth = 20;
    int search_back_depth = 10;
    double distance_warning = 3.;
  };

  using Parent = Path<RCGraph>;

  // Basic graph typedefs
  using Graph = RCGraph;
  using Edge = RCEdge;
  using Vertex = RCVertex;

  // Often used typedefs
  using vid_t = Vertex::IdType;
  using seq_t = Parent::SequenceType;
  using Parent::tf_t;

  using UniqueLock = std::unique_lock<std::recursive_mutex>;
  using LockGuard = std::lock_guard<std::recursive_mutex>;

  PTR_TYPEDEFS(LocalizationChain);

  LocalizationChain(const Config &config, const Graph::Ptr &graph)
      : Parent(graph), graph_(graph), config_(config) {}

  LocalizationChain(const Graph::Ptr &graph)
      : LocalizationChain(Config(), graph) {}

  bool isLocalized() const { return is_localized_; }

  const unsigned &trunkSequenceId() const { return trunk_sid_; }
  const unsigned &branchSequenceId() const { return branch_sid_; }

  const vid_t &petioleVertexId() const { return petiole_vid_; }
  const vid_t &twigVertexId() const { return twig_vid_; }
  const vid_t &branchVertexId() const { return branch_vid_; }
  const vid_t &trunkVertexId() const { return trunk_vid_; }

  const tf_t T_leaf_petiole() const { return T_leaf_petiole_; }
  const tf_t T_leaf_twig() const { return T_leaf_petiole_ * T_petiole_twig_; }
  const tf_t T_leaf_trunk() const {
    return T_leaf_petiole_ * T_petiole_twig_ * T_twig_branch_ * T_branch_trunk_;
  }
  const tf_t T_twig_branch() const { return T_twig_branch_; }
  const tf_t T_twig_trunk() const { return T_twig_branch_ * T_branch_trunk_; }
  const tf_t T_branch_trunk() const { return T_branch_trunk_; }
  const tf_t T_petiole_trunk() const {
    return T_petiole_twig_ * T_twig_branch_ * T_branch_trunk_;
  }

  /// What is the privileged vehicle pose (relative to the start of the path)
  tf_t T_start_twig() { return pose(trunk_sid_) * T_twig_trunk().inverse(); }
  tf_t T_start_petiole() {
    return pose(trunk_sid_) * T_petiole_trunk().inverse();
  }
  tf_t T_start_leaf() { return pose(trunk_sid_) * T_leaf_trunk().inverse(); }
  tf_t T_start_trunk() { return pose(trunk_sid_); }

  tf_t T_trunk_target(unsigned seq_id) const;

  /** \brief Resets localization chain to its initial state. */
  void reset();

  /**
   * \brief Resets the vertex we think we're the closest to and unset
   * is_localized status.
   */
  void resetTrunk(unsigned trunk_sid);

  /** \brief Updates T_leaf_twig from odometry */
  void updatePetioleToLeafTransform(const tf_t &T_leaf_twig,
                                    const bool search_closest_trunk,
                                    const bool look_backwards = false);

  /** \brief update T_twig_branch if we just localized a keyframe */
  void updateBranchToTwigTransform(const tf_t &T_twig_branch,
                                   const bool search_closest_trunk,
                                   const bool look_backwards = false);

  /** \brief Updates T_leaf_twig from odometry */
  void setPetiole(vid_t petiole_id);

  /**
   * \brief Move the localization chain forward.
   * \details
   *  Tw-->-->--Pe-->Le            >-->-->--Tw,Pe-->Le
   *  |                    ===>    |
   *  Br-->-->-->-->--Tr           >-->-->-->-->--Br,Tr-->
   */
  void convertPetioleTrunkToTwigBranch();

  /** \brief const accessor for the configuration */
  const Config &config() { return config_; }

  /** \brief Acquires a lock object that blocks modifications. */
  UniqueLock guard() const { return UniqueLock(mutex_); }
  /** \brief Manually locks the chain. */
  void lock() const { mutex_.lock(); }
  /** \brief Manually unlocks the chain. */
  void unlock() const { mutex_.unlock(); }
  /** \brief Get a reference to the mutex */
  std::recursive_mutex &mutex() const { return mutex_; }

 protected:
  /** \brief Initializes privileged path to localize against. */
  void initSequence();

  /** \brief Update the Trunk to the closest vertex. */
  void searchClosestTrunk(bool look_backwards);

  /** \brief important indices */
  unsigned trunk_sid_ = (unsigned)-1;
  unsigned branch_sid_ = (unsigned)-1;

  /** \brief important vertices (see description at the top) */
  vid_t petiole_vid_ = vid_t::Invalid();
  vid_t twig_vid_ = vid_t::Invalid();
  vid_t branch_vid_ = vid_t::Invalid();
  vid_t trunk_vid_ = vid_t::Invalid();

  /** \brief important transforms (default to identity with zero cov) */
  tf_t T_leaf_petiole_ = tf_t(true);  // frame-to-kf
  tf_t T_petiole_twig_ = tf_t(true);  // Autonomous edges
  tf_t T_twig_branch_ = tf_t(true);   // Localization
  tf_t T_branch_trunk_ = tf_t(true);  // Privileged edges

  /** \brief pose graph pointer */
  Graph::Ptr graph_;

  /** \brief configuration */
  Config config_;

  /** \brief localization status */
  bool is_localized_ = false;

  /** \brief for thread safety, use whenever read from/write to the chain */
  mutable std::recursive_mutex mutex_;
};

}  // namespace pose_graph
}  // namespace vtr

std::ostream &operator<<(std::ostream &stream,
                         const vtr::pose_graph::LocalizationChain &chain);
