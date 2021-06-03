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

// (Le) Leaf    --- The latest frame that's currently being processed
// (Pe) Petiole --- The latest live vertex.
// (Tr) Trunk   --- The privileged vertex that is estimated closest to the Leaf
// (Tw) Twig    --- The live vertex that is part of the current localization
//                  problem.
// (Br) Branch  --- The privileged vertex that is part of the current
//                  localization problem.

// ===          --- Temporal edge, these vertices are adjacent in time
// |            --- Spatial edge, these vertices have been localized to each
//                  other
// xx           --- T_Br_Tr, the cumulative transform using the last
//                  localization
// **           --- T_Le_Tr, T_Le_Br * T_Br_Tr (cumulative plus VO as a prior)

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

  LocalizationChain(const Config &config, const Graph::Ptr &graph)
      : Parent(graph), graph_(graph), config_(config) {}

  LocalizationChain(const Graph::Ptr &graph)
      : LocalizationChain(Config(), graph) {}

  inline bool isLocalized() const { return is_localized_; }

  inline const unsigned &trunkSequenceId() const { return trunk_sid_; }

  inline const vid_t &petioleVertexId() const { return petiole_vid_; }
  inline const vid_t &twigVertexId() const { return twig_vid_; }
  inline const vid_t &branchVertexId() const { return branch_vid_; }
  inline const vid_t &trunkVertexId() const { return trunk_vid_; }

  inline const tf_t T_leaf_petiole() const { return T_leaf_petiole_; }
  inline const tf_t T_leaf_twig() const {
    return T_leaf_petiole_ * T_petiole_twig_;
  }
  inline const tf_t T_leaf_trunk() const {
    return T_leaf_petiole_ * T_petiole_twig_ * T_twig_branch_ * T_branch_trunk_;
  }
  inline const tf_t T_twig_branch() const { return T_twig_branch_; }
  inline const tf_t T_twig_trunk() const {
    return T_twig_branch_ * T_branch_trunk_;
  }
  inline const tf_t T_branch_trunk() const { return T_branch_trunk_; }
  inline const tf_t T_petiole_trunk() const {
    return T_petiole_twig_ * T_twig_branch_ * T_branch_trunk_;
  }

  /// What is the privileged vehicle pose (relative to the start of the path)
  tf_t T_start_leaf() { return pose(trunk_sid_) * T_leaf_trunk().inverse(); }
  tf_t T_start_trunk() { return pose(trunk_sid_); }

  tf_t T_trunk_target(unsigned seq_id) const;

  /**
   * \brief Reset the vertex we think we're the closest to, unset is_localized
   * status
   */
  void resetTrunk(unsigned trunk_sid);

  /** \brief Updates T_leaf_twig from odometry */
  /// optionally set the a new id of the live twig if there's a new vertex
  void updatePetioleToLeafTransform(const tf_t &T_leaf_twig,
                                    bool look_backwards);

  /** \brief update T_twig_branch if we just localized a keyframe */
  void updateBranchToTwigTransform(const tf_t &T_twig_branch,
                                   bool look_backwards);

  /** \brief Updates T_leaf_twig from odometry */
  void setPetiole(vid_t petiole_id, bool first_frame = false);

  /**
   * \brief Move the localization chain forward.
   * \details
   *  Tw-->-->--Pe-->Le            >-->-->--Tw,Pe-->Le
   *  |                    ===>    |
   *  Br-->-->-->-->--Tr           >-->-->-->-->--Br,Tr-->
   */
  void convertPetioleTrunkToTwigBranch();

  /// const accessor for the configuration
  const Config &config() { return config_; }

 protected:
  // Privileged path to localize against
  void initSequence();

  // Update the Trunk to the closest vertex
  void searchClosestTrunk(bool look_backwards);

  // Important indices
  unsigned trunk_sid_ = (unsigned)-1;
  unsigned branch_sid_ = (unsigned)-1;

  vid_t petiole_vid_ = vid_t::Invalid();
  vid_t twig_vid_ = vid_t::Invalid();
  vid_t branch_vid_ = vid_t::Invalid();
  vid_t trunk_vid_ = vid_t::Invalid();

  // Important transforms
  tf_t T_leaf_petiole_;  // frame-to-kf
  tf_t T_petiole_twig_;  // Autonomous edges
  tf_t T_twig_branch_;   // Localization
  tf_t T_branch_trunk_;  // Privileged edges

  // Graph
  Graph::Ptr graph_;

  // status
  bool is_localized_ = false;

  Config config_;
};

}  // namespace pose_graph
}  // namespace vtr

std::ostream &operator<<(std::ostream &stream,
                         const vtr::pose_graph::LocalizationChain &chain);
