#pragma once

#include <lgmath/se3/Transformation.hpp>

#include <vtr_navigation/pipelines/branch_pipeline.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_vertex.hpp>

namespace vtr {
namespace navigation {

class MergePipeline : public BranchPipeline {
 public:
  PTR_TYPEDEFS(MergePipeline)

  MergePipeline(BasicTactic *_tactic)
      : BranchPipeline(_tactic),
        locSuccesses_(0),
        framesSinceLoc_(0),
        framesSinceKf_(0),
        force_keyframe_(false),
        prev_live_id_(VertexId::Invalid()) {}

  void convertData(QueryCachePtr q_data, MapCachePtr m_data) override;

  KeyframeRequest processData(QueryCachePtr q_data, MapCachePtr m_data,
                              bool first_frame) override;

  void processKeyFrame(QueryCachePtr q_data, MapCachePtr m_data,
                       bool first_frame) override;

  void makeKeyFrame(QueryCachePtr q_data, MapCachePtr m_data,
                    bool first_frame) override;

 protected:
  /// Where to move the trunk vertex when we encounter a localization failure
  virtual uint32_t _getNextTrunkSeq();

  /// Update the localization with respect to the current run
  virtual void _updateRunLoc(QueryCachePtr q_data, MapCachePtr m_data);

  /// Update the localization with respect to the privileged chain
  virtual void _updateTrunkLoc();

  /** A mutex to protect access to the localization chain */
  std::mutex chain_update_mutex_;

  int locSuccesses_;

  int framesSinceLoc_;

  int framesSinceKf_;

  EdgeTransform T_leaf_petiole_initial_;

  bool force_keyframe_;

  VertexId prev_live_id_;
};

}  // namespace navigation
}  // namespace vtr
