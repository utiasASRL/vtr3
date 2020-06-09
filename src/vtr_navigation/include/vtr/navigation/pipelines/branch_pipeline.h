#pragma once

// #include <vtr/navigation/assemblies/base_assembly.h>
#include <vtr/navigation/assemblies.h>
#include <vtr/navigation/pipelines/base_pipeline.h>
#include <vtr/navigation/tactics/basic_tactic.h>
// #include <vtr/navigation/types.h>

#include <asrl/common/timing/TimeUtils.hpp>
#include <asrl/common/utils/CommonMacros.hpp>
// #include <asrl/pose_graph/index/RCVertex.hpp>
// #include <lgmath.hpp>

namespace asrl {
namespace navigation {

class BranchPipeline : public BasePipeline {
 public:
  PTR_TYPEDEFS(BranchPipeline)

  BranchPipeline(BasicTactic* _tactic) : BasePipeline(_tactic) {
    // copy the cache data from the old tactic to this one
    auto old_pipeline = tactic->pipeline();
    auto casted_pipeline =
        std::dynamic_pointer_cast<BranchPipeline>(old_pipeline);
    if (casted_pipeline == nullptr) {
      return;
    } else {
      if (casted_pipeline->trajectory_ != nullptr) {
        trajectory_ = casted_pipeline->trajectory_;
        trajectory_time_point_ = casted_pipeline->trajectory_time_point_;
      }
      if (casted_pipeline->candidate_q_data != nullptr &&
          casted_pipeline->candidate_m_data != nullptr) {
        candidate_q_data = casted_pipeline->candidate_q_data;
        candidate_m_data = casted_pipeline->candidate_m_data;
      }
    }

    // initialised the forced kfs printing
    last_forced_kf_printed_ = std::chrono::system_clock::now();
    num_forced_kf_logged_ = 0;
  }

  /** \brief Run the image converter
   */
  virtual void convertData(QueryCachePtr q_data, MapCachePtr m_data);

  /** \brief Run the processing pipeline
   */
  virtual KeyframeRequest processData(QueryCachePtr q_data, MapCachePtr m_data,
                                      bool first_frame);
  virtual void processKeyFrame(QueryCachePtr q_data, MapCachePtr m_data,
                               bool first_frame);
  virtual void makeKeyFrame(QueryCachePtr q_data, MapCachePtr m_data,
                            bool first_frame);

#if 0
  /**
   * Assess terrain.
   */
  virtual void assessTerrain(QueryCachePtr q_data, MapCachePtr m_data, bool ta_parallelization, std::future<void>& ta_thread_future);

  void computeT_0_q(QueryCachePtr q_data, MapCachePtr m_data);

  // pipeline specific functions
  /// @brief Given that a candidate keyframe exists, turn it into an actual keyframe
  virtual void makeKeyframeFromCandidate();
  /// @brief force add a Keyframe to the graph
  void forceKeyframe(QueryCachePtr q_data, MapCachePtr m_data);
  /// @brief setup for re-running processData on a set of query and map data that has failed a vertex creation test
  void reprocessData(QueryCachePtr q_data, MapCachePtr m_data, bool first_frame);
#endif

  /** \brief Predict the transform from the keyframe time to the current frame
   */
  EdgeTransform estimateTransformFromKeyframe(
      const robochunk::std_msgs::TimeStamp& kf_stamp,
      const robochunk::std_msgs::TimeStamp& curr_stamp,
      bool check_expiry = true);

  virtual const QueryCachePtr candidateQueryCache() const {
    return candidate_q_data;
  }
  virtual const MapCachePtr candidateMapCache() const {
    return candidate_m_data;
  }

 protected:
#if 0
  /// a mutex to protect access to the localization chain
  std::mutex chain_update_mutex_;
#endif

  /** \brief: A pointer to a candidate query data to use when a keyframing test
   * fails.
   */
  QueryCachePtr candidate_q_data;

  /** \brief: a pointer to a candidate map data to use when a keyframing test
   * fails.
   */
  MapCachePtr candidate_m_data;

 private:
  /// a pointer to a trjacetory estimate so that the transform can be estimated
  /// at a future time
  std::shared_ptr<steam::se3::SteamTrajInterface> trajectory_;
  /// the time at which the trajectory was estimated
  common::timing::time_point trajectory_time_point_;

#if 0
  /// @brief: a copy of the previous T_q_m estimate
  //std::shared_ptr<lgmath::se3::TransformationWithCovariance> T_q_m_prev_;;

  /// @brief: a cached transform to the base of this run
  std::shared_ptr<std::pair<VertexId, lgmath::se3::Transformation>> T_0_trunk_cache_;

#endif
  /// @brief: last time a force keyframe was printed. This is to reduce printing
  /// on lots of errors
  std::chrono::time_point<std::chrono::system_clock> last_forced_kf_printed_;
  unsigned num_forced_kf_logged_;
};

}  // namespace navigation
}  // namespace asrl
