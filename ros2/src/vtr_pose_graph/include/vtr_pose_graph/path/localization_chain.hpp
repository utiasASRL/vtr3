#pragma once

#include <algorithm>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#include <vtr_pose_graph/path/path.hpp>

namespace vtr {
namespace pose_graph {

// A description of the localization chain
// ---------------------------------------

// Live   |> === |> === |> === Tw ==|>==|>==Pe .. Le
//               |      |       x                *
//               |      |       x              *
// Priv.  |> === |> === |> === Br ==|>==|>=== Tr

// (Le) Leaf  ---The latest frame that's currently being processed
// (Pe) Petiole --- The latest live vertex.
// (Tr) Trunk --- The privileged vertex that is estimated closest to the Leaf
// (Tw) Twig --- The live vertex that is part of the current localization
//               problem.
// (Br) Branch --- The privileged vertex that is part of the current
//                 localization problem.

// ===        ---Temporal edge, these vertices are adjacent in time
// |          ---Spatial edge, these vertices have been localized to each other
// xx         ---T_Br_Tr, the cumulative transform using the last localization
// **         ---T_Le_Tr, T_Le_Br * T_Br_Tr (cumulative plus VO as a prior)

class LocalizationChain : public Path<RCGraph> {
 public:
  // Localization Chain Configuration
  struct Config {
    double min_cusp_distance = 1.5;
    double angle_weight = 7.0;
    int search_depth = 20;
    int search_back_depth = 10;
    double distance_warning = 3.;
  };

  // Basic graph/path typedefs
  using graph_t = RCGraph;
  using edge_t = RCEdge;
  using vertex_t = RCVertex;
  using path_t = Path<graph_t>;

  // Often used typedefs
  using vid_t = vertex_t::IdType;
  using seq_t = path_t::SequenceType;
  using path_t::tf_t;

  LocalizationChain(const Config &config, const graph_t::Ptr &graph)
      : config_(config),
        path_t(graph),
        graph_(graph),
        is_localized_(false),
        trunk_sid_(0),
        twig_vid_(vid_t::Invalid()) {
  }

  LocalizationChain(const graph_t::Ptr &graph)
      : LocalizationChain(Config(), graph) {
  }

  /// Reset the vertex we think we're the closest to, unset is_localized status
  void resetTrunk(unsigned trunk_sid);

  /// Successful localization! T_twig_branch=1 if we just localized a keyframe
  void setLocalization(const tf_t &T_twig_branch, bool look_backwards);

  /// Updates live_twig->leaf from VO,
  /// optionally set the a new id of the live twig if there's a new vertex
  void updateVO(const tf_t &T_leaf_twig, bool look_backwards);

  /// Get the closest (petiole) vertex Id
  inline const vid_t &petioleVertexId() const {
    return petiole_vid_;
  }
  /// Get the closest (twig) vertex Id
  inline const vid_t &twigVertexId() const {
    return twig_vid_;
  }
  /// Get the closest (branch) vertex Id
  inline const vid_t &branchVertexId() const {
    return branch_vid_;
  }
  /// Get the closest (trunk) vertex Id
  inline const vid_t &trunkVertexId() const {
    return trunk_vid_;
  }
  /// Get the closest (trunk) sequence Id
  inline const unsigned &trunkSequenceId() const {
    return trunk_sid_;
  }
  /// Are we localized?
  inline bool isLocalized() const {
    return is_localized_;
  }
  /// What is the privileged vehicle pose (relative to the start of the path)
  tf_t T_start_leaf() {
    return pose(trunk_sid_) * T_leaf_trunk_.inverse();
  }
  tf_t T_start_trunk() {
    return pose(trunk_sid_);
  }
  /// What is the predicted localization transform
  inline const tf_t &T_leaf_trunk() const {
    return T_leaf_trunk_;
  }

  inline const tf_t &T_leaf_petiole() const {
    return T_leaf_petiole_;
  }
  /// What is the predicted localization transform
  inline const tf_t &T_leaf_twig() const {
    return T_leaf_twig_;
  }

  /// What is the predicted localization transform
  inline const tf_t &T_twig_trunk() const {
    return T_twig_trunk_;
  }

  inline const tf_t &T_branch_trunk() const {
    return T_branch_trunk_;
  }
  inline const tf_t &T_twig_branch() const {
    return T_twig_branch_;
  }
  tf_t T_trunk_target(unsigned seq_id) const;

  inline const tf_t T_petiole_trunk() const {
    return T_petiole_twig_ * T_twig_trunk_;
  }

  void convertLeafToPetiole(vid_t petiole_id, bool first_frame = false);

  //  Tw-->-->--Pe-->Le
  //  |
  //  Br-->-->-->-->--Tr

  //  >-->-->--Tw,Pe-->Le
  //  |
  //  >-->-->-->-->--Br,Tr-->
  void resetTwigAndBranch();

  void resetBranchToSid(unsigned branch) {
    T_branch_trunk_ = pose(branch).inverse() * pose(trunk_sid_);
    T_twig_branch_ = T_twig_trunk_ * T_branch_trunk_.inverse();

    branch_sid_ = branch;
    branch_vid_ = sequence_[branch];
  }

  /// const accessor for the configuration
  const Config &config() {
    return config_;
  }

 protected:
  // Privileged path to localize against
  void initSequence();

  // Update the Trunk to the closest vertex
  void searchClosestTrunk(bool look_backwards);

  Config config_;

  // Important indices
  unsigned trunk_sid_;
  unsigned branch_sid_;

  vid_t petiole_vid_;
  vid_t trunk_vid_;
  vid_t twig_vid_;
  vid_t branch_vid_;

  // Important transforms

  // frame-to-kf VO.
  tf_t T_leaf_petiole_;

  // Autonomous Edges
  tf_t T_petiole_twig_;

  // Privileged edges.
  tf_t T_branch_trunk_;

  // Localization
  tf_t T_twig_branch_;

  // Cached transforms
  tf_t T_leaf_twig_;
  tf_t T_leaf_trunk_;
  tf_t T_twig_trunk_;

  // graph and path
  graph_t::Ptr graph_;

  // status
  bool is_localized_;
};

}  // namespace pose_graph
}  // namespace vtr

std::ostream &operator<<(std::ostream &stream,
                         const vtr::pose_graph::LocalizationChain &chain);
