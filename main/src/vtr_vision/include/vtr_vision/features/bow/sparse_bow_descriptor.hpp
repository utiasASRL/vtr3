// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file sparse_bow_descriptor.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <list>
#include <map>
#include <numeric>
#include <opencv2/features2d/features2d.hpp>

#include <vtr_vision/types.hpp>

namespace vtr {
namespace vision {

/// This creates sparse Bag-of-Words descriptors that are indexed by
/// WordId, and point to counts of the number of times that word was seen.
/// It stores the vocabulary, and provides utility functions for descriptors.
template <typename WordId_>
class SparseBOWDescriptor {
 public:
  typedef WordId_ WordId;
  typedef unsigned WordCount;
  typedef std::map<WordId, WordCount> SparseBOW;
  typedef std::pair<WordId, WordCount> SparseWordCount;

  /// Construct the descriptor parent class with a vocabulary and some settings.
  SparseBOWDescriptor(
      cv::DescriptorMatcher* vocabulary,  ///< We need a vocabulary to be able
                                          ///< to create new descriptors
      float cluster_size  ///< This is the cluster size we're using (to cluster
                          ///< unmatched features)
      )
      : vocabulary_(vocabulary), cluster_size_(cluster_size) {}

  /// Compute a BoW descriptor using the member vocabulary, and flagging any
  /// unmatched features
  SparseBOW compute(
      const cv::Mat& descriptors,  ///< The descriptors we want to BoW-ify
      std::list<unsigned>* invalid_idx =
          nullptr  ///< [out] The descriptors that were further than
                   ///< cluster_size from the vocab
  ) const;

  /// Calculate the cosine distance = 1 - (a . b)/(|a| |b|) between two BoW
  /// descriptors
  static double distance(const SparseBOW& a, const SparseBOW& b);

  /// Set union two BoW descriptors: combined = combined U added
  static void addInPlace(SparseBOW& combined, const SparseBOW& added);
  /// Set union two BoW descriptos: return = a U b
  static SparseBOW add(const SparseBOW& a, const SparseBOW& b);

  /// Set subtract two BoW descriptors: combined = combined / subtracted
  static void subtractInPlace(SparseBOW& combined, const SparseBOW& subtracted);

 protected:
  /// Descriptor matcher
  cv::DescriptorMatcher* vocabulary_;

  /// Cluster size (invalid word further than this)
  float cluster_size_;
};

template <typename WordId_>
void SparseBOWDescriptor<WordId_>::addInPlace(SparseBOW& combined,
                                              const SparseBOW& added) {
  typename SparseBOW::iterator cit = combined.begin();
  typename SparseBOW::const_iterator ait = added.begin();
  while (ait != added.end()) {
    // catch cit up to eit
    while (cit != combined.end() && cit->first < ait->first) ++cit;
    // we finished looking through combined, done the loop
    if (cit == combined.end()) break;
    // if they are the same index, sum the counts
    if (cit->first == ait->first) cit->second += (ait++)->second;
    // we don't have this one, just insert it
    while (ait != added.end() && ait->first < cit->first)
      combined.emplace_hint(cit, *(ait++));
  }
  // if we had any left that weren't merged, insert them
  combined.insert(ait, added.end());
}

template <typename WordId_>
typename SparseBOWDescriptor<WordId_>::SparseBOW
SparseBOWDescriptor<WordId_>::add(const SparseBOW& a, const SparseBOW& b) {
  SparseBOW combined = a;
  addInPlace(combined, b);
  return combined;
}

template <typename WordId_>
void SparseBOWDescriptor<WordId_>::subtractInPlace(
    SparseBOW& combined, const SparseBOW& subtracted) {
  typename SparseBOW::iterator cit = combined.begin();
  typename SparseBOW::const_iterator sit = subtracted.begin();
  while (sit != subtracted.end()) {
    // catch cit up to sit
    while (cit != combined.end() && cit->first < sit->first) ++cit;
    if (cit == combined.end()) {
      // we finished looking through combined, done the loop
      break;
    } else if (cit->first == sit->first) {
      // we need to subtract from this index
      // this shouldn't happen if subtracted is a subset of combined, but check
      if (cit->second < sit->second)
        throw std::logic_error("Bow subtraction went negative.");
      cit->second -= std::min(cit->second, sit->second);
      // remove the entry if it has gone to zero
      if (cit->second == 0) combined.erase(cit++);
    } else if (cit->first > sit->first) {
      // combined didn't have this index, this shouldn't happen, but check
      throw std::logic_error("Bow subtraction went negative.");
    }
    ++sit;
  }
  // Make sure we were able to subtract all the counts
  if (sit != subtracted.end())
    throw std::logic_error("Bow subtraction went negative.");
}

template <typename WordId_>
typename SparseBOWDescriptor<WordId_>::SparseBOW
SparseBOWDescriptor<WordId_>::compute(const cv::Mat& descriptors,
                                      std::list<unsigned>* invalid_idx) const {
  // Query size
  unsigned n = descriptors.rows;

  // Allocate output
  SparseBOW bow;

  // Check for null input
  if (n == 0) return bow;

  // Match to vocabulary
  std::vector<cv::DMatch> matches;
  vocabulary_->match(descriptors, matches);

  // Check for words that were too far, and copy to sparse output
  for (unsigned i = 0; i < matches.size(); ++i) {
    const cv::DMatch& imatch = matches[i];
    if (imatch.distance > cluster_size_) {
      // record the invalid query (too far from a cluster)
      if (invalid_idx) invalid_idx->push_back(i);
      // increment the invalid word
      //++bow[uint64_t(-1)]; // < Let them do this, too unclear in here
    } else {
      // initialize or increment the word in the descriptor
      ++bow[imatch.trainIdx];
    }
  }

  return bow;
}

template <typename WordId_>
double SparseBOWDescriptor<WordId_>::distance(const SparseBOW& a,
                                              const SparseBOW& b) {
  typedef SparseBOW SB_t;
  typedef typename SB_t::const_iterator SB_cit;

  // sums for cosine distance
  auto ssum_lambda = [](double cumulative, const SparseWordCount& val) {
    return cumulative + val.second * val.second;
  };
  double ssum_a = std::accumulate(a.begin(), a.end(), double(0), ssum_lambda);
  double ssum_b = std::accumulate(b.begin(), b.end(), double(0), ssum_lambda);
  double denom = std::sqrt(ssum_a) * std::sqrt(ssum_b);
  if (denom == 0) return 0;

  double dot_prod = 0.;

  SB_cit ia = a.begin(), ib = b.begin();

  while (ia != a.end() && ib != b.end()) {
    // If the word is the same, dot product and continue
    if (ia->first == ib->first) {
      dot_prod += ia->second * ib->second;
      ++ib;
      ++ia;
      continue;
    }

    // If they're on different words, increment the lowest as much as possible
    while (ia != a.end() && ia->first < ib->first) ++ia;
    while (ib != b.end() && ib->first < ia->first) ++ib;
  }
  // Cosine distance is the complement, so that 1 is the worst, and 0 is perfect
  return 1. - dot_prod / denom;
}

extern template class SparseBOWDescriptor<unsigned>;

}  // namespace vision
}  // namespace vtr
