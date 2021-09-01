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
 * \file sliding_window_bow_descriptor.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_vision/features/bow/sparse_bow_descriptor.hpp>

namespace vtr {
namespace vision {

/// This class maintains a sliding window BoW descriptor.
/// New single-image descriptors are added to the window, appended to the
/// grouped descriptor, and old ones are popped off and subtracted.
template <typename WordId_>
class SlidingWindowBOWDescriptor {
 public:
  typedef WordId_ WordId;
  typedef SparseBOWDescriptor<WordId> SBD;
  typedef typename SBD::SparseBOW SB;

  /// We just need to know the window size
  SlidingWindowBOWDescriptor(unsigned window_size)
      : window_size_(window_size) {}

  /// Push a new BoW descriptor into the window
  template <typename SB_>
  void push(SB_&& bow_in  /// < the new BoW descriptor to add
  ) {
    // add the new one to the list
    bow_list_.emplace_back(std::forward<SB_>(bow_in));
    // merge it with the window descriptor
    SBD::addInPlace(window_bow_, bow_list_.back());
    // pop if we've exceeded the window size
    if (bow_list_.size() > window_size_) pop();
  }

  /// Get the window BoW descriptor
  const SB& bow() { return window_bow_; }

  /// Clear out the window so we can start fresh
  void clear() {
    window_bow_.clear();
    bow_list_.clear();
  }

 protected:
  /// Remove the oldest descriptor that's being popped out of the window
  void pop() {
    SBD::subtractInPlace(window_bow_, bow_list_.front());
    bow_list_.pop_front();
  }

  /// The list of single-image descriptors in the window
  std::list<SB> bow_list_;

  /// The window grouped BoW descriptor
  SB window_bow_;

  /// The size of window we're using for the group
  unsigned window_size_;

  /// Cluster size (invalid word further than this)
  float cluster_size_;
};

extern template class SlidingWindowBOWDescriptor<unsigned>;

}  // namespace vision
}  // namespace vtr
