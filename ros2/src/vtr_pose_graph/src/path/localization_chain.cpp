#include <vtr_pose_graph/evaluator/accumulators.hpp>
#include <vtr_pose_graph/evaluator/common.hpp>
#include <vtr_pose_graph/path/localization_chain.hpp>

namespace vtr {
namespace pose_graph {

// Set the vertex we think we're the closest to, unsets localized status
void LocalizationChain::resetTrunk(unsigned trunk_sid) {
  // Are we still localized? (i.e. did the vertex id not change)
  is_localized_ = is_localized_ && trunk_sid < sequence_.size() &&
                  sequence_[trunk_sid] == trunk_vid_;
  // Reset all the ids appropriately
  trunk_sid_ = trunk_sid < sequence_.size() ? trunk_sid : -1;
  trunk_vid_ =
      trunk_sid_ == unsigned(-1) ? vid_t::Invalid() : sequence_[trunk_sid_];
  branch_vid_ = trunk_vid_;
  branch_sid_ = trunk_sid_;
  T_branch_trunk_ = tf_t(true);
}

// We have to initialize when we get a new path sent to us...
void LocalizationChain::initSequence() {
  Path::initSequence();
  // reset the trunk ids to the start of the path
  resetTrunk(0);
  // unset the twig vertex id
  twig_vid_ = vid_t::Invalid();
}

// Successful localization! Woot
void LocalizationChain::setLocalization(const tf_t &T_twig_branch,
                                        bool search_backwards) {
  // update localization
  T_twig_branch_ = T_twig_branch;

  // localization to most recent vertex (maybe the same as leaf...)
  T_twig_trunk_ = T_twig_branch_ * T_branch_trunk_;

  // update the T_leaf_trunk
  T_leaf_trunk_ = T_leaf_twig_ * T_twig_trunk_;

  // Localized!
  is_localized_ = true;

  // Search along the path for the closest vertex (Trunk)
  searchClosestTrunk(search_backwards);
}

void LocalizationChain::updateVO(const tf_t &T_leaf_petiole,
                                 bool search_backwards) {
  // update vo transform and cached
  T_leaf_petiole_ = T_leaf_petiole;
  T_leaf_twig_ = T_leaf_petiole_ * T_petiole_twig_;
  // Don't need to worry about the rest if we aren't localized
  if (!is_localized_) return;

  T_leaf_trunk_ = T_leaf_twig_ * T_twig_branch_ * T_branch_trunk_;

  // Search along the path for the closest vertex (Trunk)
  searchClosestTrunk(search_backwards);
}

auto LocalizationChain::T_trunk_target(unsigned seq_id) const -> tf_t {
  // get T_0_trunk
  auto T_0_trunk = pose(trunk_sid_);
  // TODO: This isnt being properly set somewhere...
  // seq_id = sequence_.size()-1;
  auto T_0_target = pose(seq_id);
  tf_t T_trunk_target = T_0_trunk.inverse() * T_0_target;
  T_trunk_target.setCovariance(Eigen::Matrix<double, 6, 6>::Identity());
  return T_trunk_target;
}

void LocalizationChain::convertLeafToPetiole(vid_t petiole_id,
                                             bool first_frame) {
  petiole_vid_ = petiole_id;

  if (first_frame || !twig_vid_.isValid()) {
    T_petiole_twig_ = tf_t(true);
  } else {
    auto delta = graph_->breadthFirstSearch(petiole_vid_, twig_vid_);
    T_petiole_twig_ = eval::ComposeTfAccumulator(delta->begin(petiole_vid_),
                                                 delta->end(), tf_t(true));
  }

  T_leaf_petiole_ = tf_t(true);
  T_leaf_twig_ = T_petiole_twig_;
}

void LocalizationChain::resetTwigAndBranch() {
  T_twig_branch_ = T_petiole_twig_ * T_twig_branch_ * T_branch_trunk_;
  T_twig_trunk_ = T_twig_branch_;
  T_leaf_twig_ = T_leaf_petiole_;

  // Twig -> Petiole
  twig_vid_ = petiole_vid_;
  // Branch -> Trunk
  branch_vid_ = trunk_vid_;
  branch_sid_ = trunk_sid_;
  T_petiole_twig_ = tf_t(true);
  T_branch_trunk_ = tf_t(true);
}

void LocalizationChain::searchClosestTrunk(bool search_backwards) {
  // Prepare for the search
  double best_distance = std::numeric_limits<double>::max();
  double max_distance = -1.;
  double trunk_distance = std::numeric_limits<double>::max();
  unsigned best_sid = trunk_sid_;
  const unsigned int end_sid = std::min(trunk_sid_ + config_.search_depth + 1,
                                        unsigned(sequence().size()));

  // Explicit casting to avoid numerical underflow when near the beginning of
  // the chain
  const unsigned int begin_sid =
      (search_backwards
           ? unsigned(std::max(int(trunk_sid_) - config_.search_back_depth, 0))
           : trunk_sid_);

  tf_t T_trunk_root = pose(trunk_sid_).inverse(),
       T_leaf_root = T_leaf_trunk_ * T_trunk_root;

  // Find the closest vertex (updating Trunk) now that VO has updated the leaf
  for (auto path_it = begin(begin_sid); unsigned(path_it) < end_sid;
       ++path_it) {
    // T_leaf_new = T_leaf_root * T_root_new
    tf_t T_root_new = pose(path_it);
    tf_t T_leaf_new = T_leaf_root * T_root_new;

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
        LOG_EVERY_N(5, WARNING)
            << "Not searching past the cusp at " << path_it->to() << ", "
            << distance << " (m/8degress) away.";
        break;
      }
    }
  }

  // This is a temporary debug log
  LOG_IF(best_distance >= config_.distance_warning && !sequence().empty(),
         WARNING)
      << "best 'distance (m/8degrees)' is: " << best_distance
      << " sid: " << best_sid << "/" << sequence().size()
      << " vid: " << sequence_[best_sid]
      << " trunk 'distance': " << trunk_distance << " start sid: " << begin_sid
      << " end sid: " << end_sid;

  // Update if we found a closer vertex than the previous Trunk
  if (best_sid != trunk_sid_) {
    trunk_sid_ = best_sid;
    trunk_vid_ = sequence_[trunk_sid_];

    auto priv_eval =
        std::make_shared<eval::Mask::Privileged<RCGraph>::Direct>();
    priv_eval->setGraph(graph_.get());
    auto delta = graph_->dijkstraSearch(branch_vid_, trunk_vid_,
                                        eval::Weight::Const::MakeShared(1, 1),
                                        priv_eval);
    T_branch_trunk_ = eval::ComposeTfAccumulator(delta->begin(branch_vid_),
                                                 delta->end(), tf_t(true));
    T_twig_trunk_ = T_twig_branch_ * T_branch_trunk_;
    T_leaf_trunk_ = T_leaf_twig_ * T_twig_trunk_;
  }

  //  LOG(INFO) << "Update trunk: " << trunk_sid_ << " new: " <<best_sid << "
  //  dist: " <<best_distance << " first seq: " << begin_sid << " last seq: " <<
  //  end_sid
  //  << "\nT_leaf_twig: " << T_leaf_twig_.vec().transpose() << "\nT_twig_trunk:
  //  " << T_twig_trunk_.vec().transpose();
}
}  // namespace pose_graph
}  // namespace vtr

std::ostream &operator<<(std::ostream &stream,
                         const vtr::pose_graph::LocalizationChain &chain) {
  stream << " twig: " << chain.twigVertexId() << " branch "
         << chain.branchVertexId() << " trunk " << chain.trunkVertexId()
         << "\n";
  try {
    auto T_leaf_target =
        chain.T_leaf_twig() * chain.T_twig_trunk() * chain.T_trunk_target(0);
    stream << "\n T_leaf_target(0)\n" << T_leaf_target;
  } catch (std::logic_error &e) {
  }
  return stream;
}
