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
 * \file vanilla_ransac.inl
 * \brief Implementation file for the ASRL vision package
 * \details This header file defines the VanillaRansac class
 *
 * \author Kirk MacTavish, Autonomous Space Robotics Lab (ASRL)
 */
#include <omp.h>
#include <pthread.h>

#include <vtr_logging/logging.hpp>
#include <vtr_vision/outliers/sampler/basic_sampler.hpp>

namespace vtr {
namespace vision {

template <typename SolutionType>
int VanillaRansac<SolutionType>::findInliers(const SimpleMatches &matches,
                                             const ErrorList &errors,
                                             SimpleMatches *inliers) const {
  // Empty the return array
  if (!inliers) return 0;

  inliers->clear();
  inliers->reserve(errors.size());

  // Find inliers whose error is below the threshold
  for (unsigned int i = 0; i < errors.size(); ++i)
    if (errors(i) <= threshold_) inliers->push_back(matches[i]);

  // Return the number of inliers
  return inliers->size();
}

template <typename SolutionType>
int VanillaRansac<SolutionType>::run(const SimpleMatches &matches,
                                     SolutionType *model,
                                     SimpleMatches *inliers) const {
  // Verify the callback/sampler are valid
  if (!cb_ || !sampler_) return 0;
  // Ensure there's an output
  if (!model && !inliers) return 0;

  // if we are doing local optimisation, add a second internal iteration
  unsigned internal_its = enable_local_opt_ ? 2 : 1;

  // Verify the input sizes
  const unsigned int &n_pts = matches.size();
  const unsigned int &n_model = cb_->getN();
  if (n_pts < n_model) return 0;

  // Initialize
  // inliers->resize(n_pts);
  double best_error = std::numeric_limits<double>::max();

  // Sampler setup
  sampler_->setInputMatches(&matches);

  // Handle the minimal case
  if (n_pts == n_model) {
    // Check if the solve works
    if (cb_->solveModel(matches, model, threshold_) <= 0) return 0;

    // All matches were inliers
    *inliers = matches;

    // We're done!
    return inliers->size();
  }

  // a thread will set the abort variable if it finds a way to break early
  bool abort = false;

  // Main ransac loop
#pragma omp parallel for num_threads(num_threads_)
  for (unsigned int i = 0; i < iterations_; ++i) {
    if (!abort) {
      // update the scheduler priority
      sched_param sch_params;
      sch_params.sched_priority = sched_get_priority_max(SCHED_BATCH);
      if (pthread_setschedparam(pthread_self(), SCHED_BATCH, &sch_params)) {
        LOG(ERROR) << "Failed to set thread scheduling : "
                   << std::strerror(errno);
      }

      // get a new sample
      SimpleMatches i_sample;
      sampler_->getSample(n_model, &i_sample);

      // initialise the sample error
      double best_sample_error = std::numeric_limits<double>::max();

      // initialise the sample matches, model and inliers
      SolutionType sample_model;

      // keep retrying the estimation using the inlier set until the error stops
      // reducing
      bool retry = true;

      // we only want to re-estimate with the inlier set once after the initial
      // estimate
      for (unsigned ii = 0; ii < internal_its && retry && !abort; ii++) {
        // solve the model using the sample
        SolutionType i_model;
        bool i_good_sample = cb_->solveModel(i_sample, &i_model, threshold_);

        // if the sample set wasn't good, break out and get another sample set
        if (!i_good_sample) {
          retry = false;
          break;
        }

        // compute errors for each sample
        ErrorList i_errors;
        double i_error;
        bool i_completed = cb_->computeError(
            matches, i_model, &i_errors, &i_error, best_sample_error, sigma_);

        // is the average error better than the best we currently have for this
        // sample?
        if (i_completed && i_error < best_sample_error) {
          // update the best error for this sample
          best_sample_error = i_error;

          // identify the inliers of this sample
          SimpleMatches i_inliers(n_pts);
          findInliers(matches, i_errors, &i_inliers);

          // Now set up to do an optimisation with the inliers as the new sample
          sample_model = i_model;
          i_sample = i_inliers;

          // is this the best error overall?
#pragma omp critical(updatebest)
          if (!abort && i_error < best_error &&
              i_inliers.size() >= inliers->size()) {
            // update the best error
            best_error = i_error;

            // Keep a record of this best set of inliers and model
            if (inliers) *inliers = i_inliers;
            if (model) *model = i_model;

            // break early if the mean error is significantly low,
            // or if the inlier ratio is significantly high
            double ratio =
                static_cast<double>(inliers->size()) / matches.size();
            if (inliers->size() > early_stop_min_inliers_ &&
                ratio >= early_stop_ratio_) {
              abort = true;
            }
          }
          // make sure we keep trying
          retry = true;
        } else {
          // don't keep trying to estimate using the inlier set
          retry = false;
        }
      }
    }
  }
  return inliers->size();
}

}  // namespace vision
}  // namespace vtr
