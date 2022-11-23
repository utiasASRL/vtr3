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
 * \file incremental_bow_trainer.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <opencv2/features2d/features2d.hpp>
#include <vtr_vision/types.hpp>

namespace vtr {
namespace vision {

/// This class trains a BoW descriptor incrementally,
/// it checks if new features are already in the vocabulary before adding them.
/// Note: right now, we haven't extensively tested the incremental part,
/// we mostly use clusterFirstByIndex, which also gives back the BoW descriptor
/// for the descriptors we asked it to cluster, in addition to the vocabulary.
class IncrementalBOWTrainer : public cv::BOWTrainer {
 public:
  /// Construct the trainer with config
  IncrementalBOWTrainer(
      double cluster_size =
          0.4,  ///< The minimum distance between words in the vocabulary
      bool first_not_mean =
          true  ///< Add distant features to the vocabulary immediately, don't
                ///< find the mean of a new cluster
  );
  virtual ~IncrementalBOWTrainer();

  /// Clear the already clustered count for the descriptors
  virtual void clear();

  /// Returns trained vocabulary (cluster centers)
  /// This function is persistent, and appends the vocabulary
  /// Note: This assumes you only pass it descriptors that didn't match against
  /// the existing vocabulary, that check is up to you!
  virtual cv::Mat cluster() const;
  /// Returns trained vocabulary (cluster centers)
  /// This function is non-persistent, and doesn't save anything to members
  virtual cv::Mat cluster(
      const cv::Mat& descriptors  ///< The descriptors we'd like to cluster into
                                  ///< a vocabulary
  ) const;

  /// Returns indices to the vocabulary, and optionally the closest cluster
  /// center for each descriptor. This is the main clustering function
  std::vector<unsigned> clusterFirstByIndex(
      const cv::Mat&
          descriptors,  ///< The descriptors to cluster into a vocabulary
      std::vector<unsigned>* closest =
          nullptr  ///< [out] optionally, the closest vocabulary word
  ) const;

 protected:
  /// Takes initial clusters (outlier descriptors that were far from previous),
  /// and takes the mean of all the descriptors assigned to that cluster.
  /// Note: we don't actually use this right now, but it makes the algorithm
  /// Maximum Separation Clustering (MSC) used by FabMap. It's slower though.
  cv::Mat refineCenters(const cv::Mat& descriptors,
                        const cv::Mat& initial_centers) const;

  /// The maximum cluster distance before a new word is created
  double cluster_size_;
  /// Whether to use the first descriptor (fast) or the mean (accurate) as the
  /// cluster center
  bool first_not_mean_;

  int already_clustered_;
  cv::Mat centers_;
};

}  // namespace vision
}  // namespace vtr
