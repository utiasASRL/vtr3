#include <vtr_navigation/pipelines/localization_search_pipeline.hpp>

namespace vtr {
namespace navigation {

/// Where to move the trunk vertex when we encounter a localization failure
uint32_t LocalizationSearchPipeline::_getNextTrunkSeq() {
  // Reset the search width if we have localized in the last 5 attempts (we must be close)
  if (locSuccesses_ > 0) {
    searchWidth_ = 2 * searchStep_;
  }

  // Flip to the other side of the trunk
  lastOffset_ *= -1;

  // Trunk sequence and max distance to either end point
  auto trunkSeq = tactic->chain_.trunkSequenceId();
  auto trunkSpan = std::max(trunkSeq, uint32_t(tactic->chain_.sequence().size()) - trunkSeq);

  // Increment on the negative offset.  This is an arbitrary choice.
  if (lastOffset_ <= 0) {
    lastOffset_ -= searchStep_;
  }

  // If the offset is too big, reset to zero without incrementing the search window
  if (std::abs(lastOffset_) > (int) trunkSpan) {
    lastOffset_ = 0;
    return trunkSeq;
  }

  // If we move past the window, increment the window width and restart from zero offset
  if (std::abs(lastOffset_) > (int) searchWidth_) {
    lastOffset_ = 0;
    searchWidth_ += searchStep_;
    return trunkSeq;
  }

  int newSeq = int(trunkSeq) + lastOffset_;

  if (newSeq < 0 || newSeq >= int(tactic->chain_.sequence().size())) {
    // Recursion: skip this sequence because it was past an endpoint.  Return the next sequence, as we haven't exceeded
    // both endpoints yet (checked above)
    return this->_getNextTrunkSeq();
  }

  return uint32_t(newSeq);
}

/// Update the localization with respect to the current run
void LocalizationSearchPipeline::_updateRunLoc(QueryCachePtr q_data, MapCachePtr m_data) {
  // Update the localization so the path tracker doesn't freak out
  tactic->updateLocalization(q_data, m_data);
}

/// Update the localization with respect to the privileged chain
void LocalizationSearchPipeline::_updateTrunkLoc() {
  tactic->updatePersistentLocalization(tactic->chain_.trunkVertexId(), tactic->chain_.T_leaf_trunk());
  LOG(INFO) << "Localization search successfully updated persistent localization at " << tactic->chain_.trunkVertexId()
            << ".";
  const lgmath::se3::TransformationWithCovariance &T_lt = tactic->chain_.T_leaf_trunk();
  if (T_lt.covarianceSet()) {
    LOG(INFO) << "Var: " << T_lt.cov().diagonal().transpose();
  }
}

}
}
