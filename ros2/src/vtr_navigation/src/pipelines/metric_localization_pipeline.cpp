
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>   //todo: update deprecated headers
#include <unistd.h>

#include <lgmath/se3/Transformation.hpp>

#include <vtr_navigation/pipelines/metric_localization_pipeline.hpp>

//#include <asrl/pose_graph/index/RCVertex.hpp>

namespace vtr {
namespace navigation {

void MetricLocalizationPipeline::convertData(QueryCachePtr q_data,
                                             MapCachePtr m_data) {
  BranchPipeline::convertData(q_data, m_data);
}

BasePipeline::KeyframeRequest MetricLocalizationPipeline::processData(
    QueryCachePtr q_data, MapCachePtr m_data, bool first_frame) {
  // Run VO
  auto is_kf = BranchPipeline::processData(q_data, m_data, first_frame);

  // check if the map is initialized and a valid T_q_m exists
  if ((m_data->map_status.is_valid() && *m_data->map_status == MAP_NEW) ||
      !(m_data->T_q_m.is_valid())) {
    return is_kf;
  }

  // Update the VO position in the localization chain
  std::lock_guard<std::mutex> lck(chain_update_mutex_);
  tactic->chain_.updateVO(*m_data->T_q_m, false);

  // only update if we did not have a vertex failure
  if (*q_data->new_vertex_flag != FAILURE) {
    tactic->updateLocalization(q_data, m_data);
    m_data->map_id = tactic->closestVertexID();
    m_data->T_q_m_prior = tactic->chain_.T_leaf_trunk();
    q_data->live_id = tactic->currentVertexID();
  } else {
    LOG(INFO) << " Not updating loc., this was a failed frame";
  }

  return is_kf;
}

void MetricLocalizationPipeline::localizeKeyFrame(QueryCachePtr q_data,
                                                  MapCachePtr m_data,
                                                  bool first_frame) {
  // Get stuff from the tactic
  const auto& loc = tactic->getLocalizer();
  const auto& pose_graph = tactic->poseGraph();

  // create a new map cache and fill it out
  MapCachePtr loc_data = std::make_shared<MapCache>();

  // the map is initialized in this configuration
  loc_data->map_status = MAP_INITIALIZED;

  // We're localizing against the closest privileged keyframe/vertex
  *loc_data->map_id = *m_data->map_id;
  if (first_frame || tactic->chain_.isLocalized() == false) {
    loc_data->T_q_m_prior.fallback(true);
    const Eigen::Matrix<double, 6, 6> loc_cov =
        tactic->config().pipeline_config.default_loc_cov;
    loc_data->T_q_m_prior->setCovariance(loc_cov);
  } else {
    loc_data->T_q_m_prior = tactic->chain_.T_twig_branch();
  }

  // Experience recognition runs in the vo pipeline, copy over the results
  if (m_data->recommended_experiences.is_valid())
    loc_data->recommended_experiences = *m_data->recommended_experiences;

  loc_data->localization_status.fallback();

  // Run the localizer against the closest vertex
  loc->run(*q_data, *loc_data, pose_graph);
  loc->updateGraph(*q_data, *loc_data, pose_graph, *q_data->live_id);

  // TODO turn this on to actually update the graph (not just the loc. chain)
  // TODO move this above the success check once we use a VO prior
  // TODO add edge if the last keyframe doesn't have one, and we're not a kf?
  // Add a spatial edge to the pose graph and update other info
  EdgeTransform T_q_m = *loc_data->T_q_m_prior;
  if (*loc_data->steam_failure || !*loc_data->success) {
    tactic->incrementLocCount(-1);
  } else {
    T_q_m = *loc_data->T_q_m;
    tactic->incrementLocCount(1);
  }

  // update the transform
  auto edge_id =
      EdgeId(*q_data->live_id, *m_data->map_id, pose_graph::Spatial);
  pose_graph::RCEdge::Ptr edge;
  if (pose_graph->contains(edge_id)) {
    edge = pose_graph->at(edge_id);
    edge->setTransform(T_q_m.inverse());
  } else {
    edge =
        pose_graph->addEdge(*q_data->live_id, *m_data->map_id, T_q_m.inverse(),
                            pose_graph::Spatial, false);
  }

  // lock the mutex and set the localization chain.
  {
    std::lock_guard<std::mutex> lck(chain_update_mutex_);
    tactic->chain_.setLocalization(T_q_m, false);
  }
}

void MetricLocalizationPipeline::processKeyFrame(QueryCachePtr q_data,
                                                 MapCachePtr m_data,
                                                 bool first_frame) {
  // do the common keyframe processing (quick vo + refined vo)
  BranchPipeline::processKeyFrame(q_data, m_data, first_frame);

  // run localization
  bool thread_available =
      keyframe_thread_future_.valid() == false ||
      keyframe_thread_future_.wait_for(std::chrono::milliseconds(0)) ==
          std::future_status::ready;

  if (thread_available || !localization_skippable_) {
    if (localization_parallelization_) {
      // *** THIS WILL BLOCK if keyframe_thread_future_ has a thread in progress
      if (keyframe_thread_future_.valid()) keyframe_thread_future_.wait();
      keyframe_thread_future_ = std::async(
          std::launch::async, &MetricLocalizationPipeline::localizeKeyFrame,
          this, q_data, m_data, first_frame);
    } else {
      localizeKeyFrame(q_data, m_data, first_frame);
    }
  } else {
    LOG(WARNING) << "Skipping localization job.";
  }
}

void MetricLocalizationPipeline::processPetiole(QueryCachePtr q_data,
                                                MapCachePtr m_data,
                                                bool first_frame) {}

void MetricLocalizationPipeline::makeKeyFrame(QueryCachePtr q_data,
                                              MapCachePtr m_data,
                                              bool first_frame) {
  BranchPipeline::makeKeyFrame(q_data, m_data, first_frame);
  std::lock_guard<std::mutex> lck(chain_update_mutex_);
  tactic->chain_.convertLeafToPetiole(*q_data->live_id, first_frame);

  // add spatial edge between the current petiole and the priv.
  const auto& pose_graph = tactic->poseGraph();
  pose_graph->addEdge(*q_data->live_id, tactic->chain_.trunkVertexId(),
                      tactic->chain_.T_leaf_trunk().inverse(),
                      pose_graph::Spatial, false);
}

void MetricLocalizationPipeline::wait() {
  // wait for the keyframe thread to join
  if (keyframe_thread_future_.valid()) {
    keyframe_thread_future_.wait();
  }
}

}  // namespace navigation
}  // namespace vtr
