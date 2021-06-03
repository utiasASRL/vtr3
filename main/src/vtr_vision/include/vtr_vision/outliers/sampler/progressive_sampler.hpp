////////////////////////////////////////////////////////////////////////////////
/// @brief Header file for the ASRL vision package
/// @details
///
/// @author Kirk MacTavish, ASRL
///////////////////////////////////////////////////////////////////////////////

#pragma once

// Internal
#include <vtr_vision/outliers/sampler/basic_sampler.hpp>

// External
#include <vector>
#include <algorithm>
#include <memory>
#include <chrono>

namespace vtr {
namespace vision {

// Forward declarations
class VerifySampleNoOp;

////////////////////////////////////////////////////////////////////
/// @brief This class provides a basic uniform match sampler
///
/// @details This class returns a uniform random sample from a list of matches
////////////////////////////////////////////////////////////////////
class ProgressiveSampler : public BasicSampler {

public:

  ////////////////////////////////////////////////////////////////////
  /// @brief Default Constructor
  ///
  /// @details This defaults to a VerifyIndices verifier
  ////////////////////////////////////////////////////////////////////
  explicit ProgressiveSampler(unsigned T_N);

  ////////////////////////////////////////////////////////////////////
  /// @brief Set member
  /// @param [in] order Match indices in order of best to worst
  ////////////////////////////////////////////////////////////////////
  void setMatchOrder(const std::vector<unsigned> * order);

  ////////////////////////////////////////////////////////////////////
  /// @brief Register the sample verifier
  /// @param [in] verifier The sample verifier to use
  ////////////////////////////////////////////////////////////////////
  virtual void setVerifier(const std::shared_ptr<VerifySampleNoOp>& verifier) {
    LOG(WARNING) << "Progressive sampler can only use VerifySampleIndices or it can get stuck. "
                 << "Reverting to VerifySampleIndices and ignoring request for "
                 << typeid(verifier).name() << ".";
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Get a sample
  /// @param [in] The sample size (number of matches included in the sample)
  /// @param [out] p_sample A vector of samples verified using the callback
  /// @param [in] max_attempts Will not try to verify more samples than this before giving up
  /// @return true if the sample is valid (did not exceed max_attempts)
  ////////////////////////////////////////////////////////////////////
  virtual bool getSample(unsigned int m, SimpleMatches* p_sample, const unsigned int& max_attempts = 1000);

private:

  ////////////////////////////////////////////////////////////////////
  /// @brief Check sample and match setup
  /// @param [in] The sample size (number of matches included in the sample)
  /// @param [in] p_sample A vector of samples verified using the callback
  /// @return true if the parameters check out
  ////////////////////////////////////////////////////////////////////
  bool precheck(unsigned int m, SimpleMatches *p_sample);

  /// @brief The order in which matches should be sampled
  const std::vector<unsigned>* order_;

  /// @brief The progressive population size to sample from
  unsigned int n_;

  /// @brief The cached sample size
  unsigned int m_;

  /// @brief The sample counter
  unsigned int t_;

  /// @brief Increase the set after this many samples (integer)
  unsigned Tp_;

  /// @brief Increase the set after this many samples
  double T_;

  /// @brief The number of samples after which it's uniform sampling
  unsigned T_N_;

};

} // namespace vision
} // namespace vtr_vision
