////////////////////////////////////////////////////////////////////////////////
/// @brief Base class to sample from matches for RANSAC
/// @details
///
/// @author Kirk MacTavish, ASRL
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <vtr_vision/types.hpp>

#include <vector>
#include <algorithm>
#include <memory>
#include <chrono>
#include <random>

namespace vtr {
namespace vision {

// Forward declarations
class VerifySampleNoOp;

////////////////////////////////////////////////////////////////////
/// @brief This class provides a basic uniform match sampler
///
/// @details This class returns a uniform random sample from a list of matches
////////////////////////////////////////////////////////////////////
class BasicSampler {

public:

  /// @brief Shared pointer type
  typedef std::shared_ptr<BasicSampler> Ptr;

  ////////////////////////////////////////////////////////////////////
  /// @brief Default Constructor
  ///
  /// @details This defaults to a VerifyIndices verifier
  ////////////////////////////////////////////////////////////////////
  BasicSampler();

  ////////////////////////////////////////////////////////////////////
  /// @brief Constructor
  /// @param [in] verifier The sample verifier to use.
  ////////////////////////////////////////////////////////////////////
  explicit BasicSampler(const std::shared_ptr<VerifySampleNoOp>& verifier);

  ////////////////////////////////////////////////////////////////////
  /// @brief Set member
  /// @param [in] matches The correspondence to sample from, owned by user
  ////////////////////////////////////////////////////////////////////
  virtual void setInputMatches(const SimpleMatches* matches);

  ////////////////////////////////////////////////////////////////////
  /// @brief Register the sample verifier
  /// @param [in] verifier The sample verifier to use
  ////////////////////////////////////////////////////////////////////
  virtual void setVerifier(const std::shared_ptr<VerifySampleNoOp>& verifier) {
    verifier_ = verifier;
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Set the seed for the random number generator
  /// @param [in] seed The seed
  ////////////////////////////////////////////////////////////////////
  void setSeed(unsigned int seed) {
    eng_.seed(seed);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Randomize the seed for the random number generator
  ////////////////////////////////////////////////////////////////////
  void randomSeed() {
    eng_.seed(std::chrono::system_clock::now().time_since_epoch().count());
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Get a sample
  /// @param [in] The sample size (number of matches included in the sample)
  /// @param [out] p_sample A vector of samples verified using the callback
  /// @param [in] max_attempts Will not try to verify more samples than this before giving up
  /// @return true if the sample is valid (did not exceed max_attempts)
  ////////////////////////////////////////////////////////////////////
  virtual bool getSample(unsigned int m, SimpleMatches* p_sample, const unsigned int& max_attempts = 1000);

protected:

  ////////////////////////////////////////////////////////////////////
  /// @brief Check sample and match setup
  /// @param [in] The sample size (number of matches included in the sample)
  /// @param [in] p_sample A vector of samples verified using the callback
  /// @return true if the parameters check out
  ////////////////////////////////////////////////////////////////////
  virtual bool precheck(unsigned int m, SimpleMatches* p_sample);

  /// @brief The callback used to verify samples
  std::shared_ptr<VerifySampleNoOp> verifier_;

  /// @brief The matches from which to sample
  const SimpleMatches* matches_;

  /// @brief Random engine
  std::default_random_engine eng_;

  /// @brief Random distribution
  std::uniform_int_distribution<int> dist_;

};

} // namespace vision
} // namespace vtr_vision
