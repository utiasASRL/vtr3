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
 * \file localization_chain.inl
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_pose_graph/path/localization_chain.hpp"

#include "vtr_pose_graph/evaluator/evaluators.hpp"
#include "vtr_pose_graph/path/accumulators.hpp"

namespace vtr {
namespace pose_graph {

template <class Graph>
EdgeTransform LocalizationChain<Graph>::T_trunk_target(unsigned seq_id) const {
  LockGuard lock(this->mutex_);
  auto T_0_trunk = this->pose(trunk_sid_);
  auto T_0_target = this->pose(seq_id);
  EdgeTransform T_trunk_target = T_0_trunk.inverse() * T_0_target;
  /// \todo should we set covariance here?
  T_trunk_target.setCovariance(Eigen::Matrix<double, 6, 6>::Identity());
  return T_trunk_target;
}

template <class Graph>
void LocalizationChain<Graph>::reset() {
  LockGuard lock(this->mutex_);
  // Reset path sequence to empty
  // this should reset trunk, branch to invalid but does not reset twig, petiole
  // and leaf.
  this->setSequence(Sequence());

  // just reset everything here again to be sure
  trunk_sid_ = (unsigned)-1;
  branch_sid_ = (unsigned)-1;

  petiole_vid_ = VertexId::Invalid();
  twig_vid_ = VertexId::Invalid();
  branch_vid_ = VertexId::Invalid();
  trunk_vid_ = VertexId::Invalid();

  // Important transforms (default to identity with zero cov)
  T_leaf_petiole_ = EdgeTransform(true);  // frame-to-kf
  T_petiole_twig_ = EdgeTransform(true);  // Autonomous edges
  T_twig_branch_ = EdgeTransform(true);   // Localization
  T_branch_trunk_ = EdgeTransform(true);  // Privileged edges

  leaf_stamp_ = -1;
  leaf_velocity_ = Eigen::Matrix<double, 6, 1>::Zero();

  is_localized_ = false;
}

template <class Graph>
void LocalizationChain<Graph>::resetTrunk(unsigned trunk_sid) {
  LockGuard lock(this->mutex_);
  // Are we still localized? (i.e. did the vertex id not change)
  is_localized_ = is_localized_ && trunk_sid < this->sequence_.size() &&
                  this->sequence_[trunk_sid] == trunk_vid_;
  // Reset all the ids appropriately
  trunk_sid_ = trunk_sid < this->sequence_.size() ? trunk_sid : -1;
  trunk_vid_ = trunk_sid_ == unsigned(-1) ? VertexId::Invalid()
                                          : this->sequence_[trunk_sid_];
  branch_vid_ = trunk_vid_;
  branch_sid_ = trunk_sid_;
  T_branch_trunk_ = EdgeTransform(true);
}

template <class Graph>
void LocalizationChain<Graph>::initSequence() {
  Parent::initSequence();
  // unset the twig vertex id
  twig_vid_ = VertexId::Invalid();
  // reset the trunk ids to the start of the path
  resetTrunk(0);
}

template <class Graph>
void LocalizationChain<Graph>::updatePetioleToLeafTransform(
    const EdgeTransform &T_leaf_petiole, const bool search_closest_trunk,
    const bool search_backwards) {
  LockGuard lock(this->mutex_);

  // update odometry transform
  T_leaf_petiole_ = T_leaf_petiole;

  if (!search_closest_trunk) return;

  // Don't need to worry about the rest if we aren't localized
  if (!is_localized_) return;

  // Search along the path for the closest vertex (Trunk)
  searchClosestTrunk(search_backwards);
}

template <class Graph>
void LocalizationChain<Graph>::updatePetioleToLeafTransform(
    const Timestamp &leaf_stamp,
    const Eigen::Matrix<double, 6, 1> &leaf_velocity,
    const EdgeTransform &T_leaf_petiole, const bool search_closest_trunk,
    const bool search_backwards) {
  LockGuard lock(this->mutex_);

  // update odometry
  leaf_stamp_ = leaf_stamp;
  leaf_velocity_ = leaf_velocity;
  T_leaf_petiole_ = T_leaf_petiole;

  if (!search_closest_trunk) return;

  // Don't need to worry about the rest if we aren't localized
  if (!is_localized_) return;

  // Search along the path for the closest vertex (Trunk)
  searchClosestTrunk(search_backwards);
}

template <class Graph>
void LocalizationChain<Graph>::setPetiole(const VertexId &petiole_id) {
  LockGuard lock(this->mutex_);
  petiole_vid_ = petiole_id;

  if (!twig_vid_.isValid() || !petiole_vid_.isValid()) {
    T_petiole_twig_ = EdgeTransform(true);
  } else {
    auto eval =
        std::make_shared<eval::mask::temporal::Eval<Graph>>(*this->graph_);
    auto delta = this->graph_->dijkstraSearch(
        petiole_vid_, twig_vid_,
        std::make_shared<eval::weight::ConstEval>(1, 1), eval);
    T_petiole_twig_ = eval::ComposeTfAccumulator(
        delta->begin(petiole_vid_), delta->end(), EdgeTransform(true));
  }

  // update odometry transform to identity (because petiole is that latest)
  T_leaf_petiole_ = EdgeTransform(true);
}

template <class Graph>
void LocalizationChain<Graph>::initializeBranchToTwigTransform(
    const EdgeTransform &T_twig_branch) {
  LockGuard lock(this->mutex_);
  // initialize localization
  T_twig_branch_ = T_twig_branch;
  is_localized_ = false;
}

template <class Graph>
void LocalizationChain<Graph>::updateBranchToTwigTransform(
    const VertexId &twig_vid, const VertexId &branch_vid,
    const unsigned &branch_sid, const EdgeTransform &T_twig_branch,
    const bool search_closest_trunk, const bool search_backwards) {
  LockGuard lock(this->mutex_);

  // find the transform from old twig to new twig
  auto Ttw_old_new = [&]() {
    if (twig_vid == twig_vid_) {
      return EdgeTransform(true);
    } else if (twig_vid == petiole_vid_) {
      return T_petiole_twig_.inverse();
    } else {
      auto eval =
          std::make_shared<eval::mask::temporal::Eval<Graph>>(*this->graph_);
      auto delta = this->graph_->dijkstraSearch(
          twig_vid_, twig_vid, std::make_shared<eval::weight::ConstEval>(1, 1),
          eval);
      return eval::ComposeTfAccumulator(delta->begin(twig_vid_), delta->end(),
                                        EdgeTransform(true));
    }
  }();

  // find the transform from old branch to new branch
  auto Tbr_old_new = [&]() {
    if (branch_vid == branch_vid_) {
      return EdgeTransform(true);
    } else if (branch_vid == trunk_vid_) {
      return T_branch_trunk_;
    } else {
      auto eval =
          std::make_shared<eval::mask::privileged::Eval<Graph>>(*this->graph_);
      auto delta = this->graph_->dijkstraSearch(
          branch_vid_, branch_vid,
          std::make_shared<eval::weight::ConstEval>(1, 1), eval);
      return eval::ComposeTfAccumulator(delta->begin(branch_vid_), delta->end(),
                                        EdgeTransform(true));
    }
  }();

  // update localization
  T_petiole_twig_ = T_petiole_twig_ * Ttw_old_new;
  T_twig_branch_ = T_twig_branch;
  T_branch_trunk_ = Tbr_old_new.inverse() * T_branch_trunk_;

  // update vertex id
  twig_vid_ = twig_vid;
  branch_vid_ = branch_vid;
  branch_sid_ = branch_sid;

  // Localized!
  is_localized_ = true;

  if (!search_closest_trunk) return;

  // Search along the path for the closest vertex (Trunk)
  searchClosestTrunk(search_backwards);
}

template <class Graph>
void LocalizationChain<Graph>::searchClosestTrunk(bool search_backwards) {
  // Prepare for the search
  double best_distance = std::numeric_limits<double>::max();
  double max_distance = -1.;
  double trunk_distance = std::numeric_limits<double>::max();
  unsigned best_sid = trunk_sid_;
  const unsigned end_sid = std::min(trunk_sid_ + config_.search_depth + 1,
                                    unsigned(this->sequence_.size()));

  // Explicit casting to avoid numerical underflow when near the beginning of
  // the chain
  const unsigned begin_sid =
      (search_backwards
           ? unsigned(std::max(int(trunk_sid_) - config_.search_back_depth, 0))
           : trunk_sid_);

  EdgeTransform T_trunk_root = this->pose(trunk_sid_).inverse();
  EdgeTransform T_leaf_root = T_leaf_trunk() * T_trunk_root;

  // Find the closest vertex (updating Trunk) now that VO has updated the leaf
  for (auto path_it = this->begin(begin_sid); unsigned(path_it) < end_sid;
       ++path_it) {
    EdgeTransform T_root_new = this->pose(path_it);
    EdgeTransform T_leaf_new = T_leaf_root * T_root_new;

    // Calculate the "distance"
    Eigen::Matrix<double, 6, 1> se3_leaf_new = T_leaf_new.vec();
    double distance = se3_leaf_new.head<3>().norm() +
                      config_.angle_weight * se3_leaf_new.tail<3>().norm();

    // This block is just for the debug log below
    if (unsigned(path_it) == trunk_sid_) trunk_distance = distance;

    // Record the best distance
    max_distance = std::max(distance, max_distance);
    if (distance < best_distance) {
      best_distance = distance;
      best_sid = unsigned(path_it);
    }

    // This block detects direction switches, and prevents searching across them
    // It's only enabled in safe-search mode (no searching backwards as well),
    // it only tries if the current sid is not an extremum of the search range,
    // and it only stops at cusps that pass X m in 'distance' from the current
    // position
    if (search_backwards == false && max_distance > config_.min_cusp_distance &&
        unsigned(path_it) > begin_sid && unsigned(path_it) + 1 < end_sid) {
      Eigen::Matrix<double, 6, 1> vec_prev_cur = path_it->T().vec();
      Eigen::Matrix<double, 6, 1> vec_cur_next = (path_it + 1)->T().vec();
      // + means they are in the same direction (note the negative at the front
      // to invert one of them)
      double r_dot = vec_prev_cur.head<3>().dot(vec_cur_next.head<3>());
      // + means they are in the same direction
      double C_dot = vec_prev_cur.tail<3>().dot(vec_cur_next.tail<3>());
      // combine the translation and rotation components using the angle weight
      double T_dot = r_dot + config_.angle_weight * C_dot;
      // If this is negative, they are in the 'opposite direction', and we're at
      // a cusp
      if (T_dot < 0) {
        CLOG_EVERY_N(1, DEBUG, "pose_graph")
            << "Not searching past the cusp at " << path_it->to() << ", "
            << distance << " (m/8degress) away.";
        break;
      }
    }
  }

  // This is a temporary debug log
  CLOG_IF(best_distance >= config_.distance_warning && !this->sequence_.empty(),
          WARNING, "pose_graph")
      << "best 'distance (m/8degrees)' is: " << best_distance
      << " sid: " << best_sid << "/" << this->sequence_.size()
      << " vid: " << this->sequence_[best_sid]
      << " trunk 'distance': " << trunk_distance << " start sid: " << begin_sid
      << " end sid: " << end_sid;

  // Update if we found a closer vertex than the previous Trunk
  if (best_sid != trunk_sid_) {
    CLOG(DEBUG, "pose_graph") << "Updating trunk_sid_";

    trunk_sid_ = best_sid;
    trunk_vid_ = this->sequence_[trunk_sid_];

    auto priv_eval =
        std::make_shared<eval::mask::privileged::Eval<Graph>>(*this->graph_);
    auto delta = this->graph_->dijkstraSearch(
        branch_vid_, trunk_vid_,
        std::make_shared<eval::weight::ConstEval>(1, 1), priv_eval);
    T_branch_trunk_ = eval::ComposeTfAccumulator(
        delta->begin(branch_vid_), delta->end(), EdgeTransform(true));
  }

  CLOG(DEBUG, "pose_graph")
      << "Update trunk to: " << trunk_vid_ << ", distance: " << best_distance
      << ", first seq: " << begin_sid << ", last seq: " << end_sid;
}

}  // namespace pose_graph
}  // namespace vtr