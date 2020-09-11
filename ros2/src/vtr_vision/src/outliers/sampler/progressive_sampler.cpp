////////////////////////////////////////////////////////////////////////////////
/// @brief This file implements the Progressive RANSAC sampler (Chum and Matas)
/// @details
///////////////////////////////////////////////////////////////////////////////

#include "vtr_vision/outliers/sampler/progressive_sampler.hpp"
#include "vtr_vision/outliers/sampler/verify_sample_indices.hpp"

// Logging
#include "vtr_logging/logging.hpp"

// External
#include <random>


namespace vtr {
namespace vision {

ProgressiveSampler::ProgressiveSampler(unsigned T_N)
  : T_N_(T_N) {
}

void ProgressiveSampler::setMatchOrder(const std::vector<unsigned>* order) {
  // Save a ref to the order
  order_ = order;

  // Reset progression counter
  n_ = 0;
  t_ = 0;
  m_ = 0;
  Tp_ = 1.;
}

bool ProgressiveSampler::getSample(unsigned int m, SimpleMatches* p_sample, const unsigned int& max_attempts) {

  // Preliminary checks
  if (!precheck(m, p_sample)) return false;

  // References
  SimpleMatches& sample = *p_sample;
  const SimpleMatches& matches = *matches_;
  const std::vector<unsigned>& order = *order_;
  const unsigned N = matches.size();

  // 1. Choice of the hypothesis generation set
  // We slightly modify PROSAC and ceiling the floating point T
  if (n_ < N) {
    // Increment fancy counters
    if (t_++ == Tp_) {
      ++n_;
      double T_prev = T_;
      T_ = double(n_+1) / double(n_+1-m) * T_;
      Tp_ = Tp_ + ceil(T_-T_prev);
    }

    // 2. Semi-random sample
    // Note, Prosac's "if T'_n < t" should be "if t < T'_n"
    sample.push_back(matches[order[n_-1]]);
    if(!verifier_->checkSubset(sample, 0, m)) {
      sample.clear();
      return false;
    }
  }

  // Restrict the domain to n (-1 if forced sample)
  dist_ = std::uniform_int_distribution<int>(0,n_-1-sample.size());

  // Loop over each index in the sample
  for (unsigned tries = 0; sample.size() < m && tries < max_attempts; ++tries) {
    // Get a new sample
    unsigned i = order[dist_(eng_)];
    if (i >= matches.size()) continue;
    sample.push_back(matches[i]);

    // Verify the sample
    if (!verifier_->checkSubset(sample, sample.size()-1, m)) {
      // Remove the rejected index
      sample.pop_back();
    } else {
      // Reset the rejection counter
      tries = 0;
    }
  }

  if (sample.size() != m) {
    // We were unsuccessful in finding a unique sample that met the criteria of the verifier
    sample.clear();
    return false;
  }

  // Done
  return true;
}

// implements "n choose k" (binomial coefficient)
unsigned long long choose(unsigned long long n, unsigned long long k) {
  if (k > n) return 0;
  unsigned long long r = 1;
  for (unsigned long long d = 1; d <= k; ++d) {
    r *= n--;
    r /= d;
  }
  return r;
}

bool ProgressiveSampler::precheck(unsigned int m, SimpleMatches *p_sample) {
  // Base checks
  if (!BasicSampler::precheck(m, p_sample)) return false;

  // Make sure we have the match order, and that it's the same size as the match list
  if (!order_ || matches_->size() != order_->size()) {
    LOG(ERROR) << "The ProgressiveSampler match order is not set properly.";
    return false;
  }

  // Check that sample size hasn't changed
  if (!t_) {
    m_ = m;
    n_ = m;
    T_ = double(T_N_)/choose(order_->size(),m); // Approx T_N/(N Choose m)
  } else if (m_ != m) {
    LOG(ERROR) << "Sample size can't change in the middle of ProgressiveSampler.";
    return false;
  }

  return true;
}

} // namespace vision
} // namespace vtr_vision
