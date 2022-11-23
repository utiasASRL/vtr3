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
 * \file sampler_tests.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include <vtr_logging/logging_init.hpp>
#include <vtr_vision/outliers.hpp>
#include <vtr_vision/type_helpers.hpp>

using namespace vtr::vision;

TEST(Vision, sampler) {
  // Settings
  unsigned N_matches = 10;       // # of matches
  unsigned N_progressive = 100;  // max # of progressive samples
  unsigned N_samples = 20;       // # of samples
  unsigned m = 3;                // pts per sample

  // Create the match pairs (just 1-1)
  SimpleMatches matches;
  matches.reserve(N_matches);
  for (unsigned int i = 0; i < N_matches; ++i)
    matches.push_back(SimpleMatch(i, i));

  // Create ordering (reverse)
  std::vector<unsigned> order(N_matches);
  for (unsigned i = 0; i < N_matches; ++i) order[i] = N_matches - i - 1;

  // Create the sampler
  auto sampler = std::make_shared<ProgressiveSampler>(N_progressive);
  sampler->setInputMatches(&matches);
  sampler->setMatchOrder(&order);

  // Collect samples
  std::vector<SimpleMatches> samples(N_samples);
  for (unsigned i = 0; i < N_samples; ++i) {
    EXPECT_TRUE(sampler->getSample(m, &samples[i], 1000));
  }

  unsigned n = m - 2;  // Start 1 earlier, allowed to increase
  for (unsigned i = 0; i < std::min(N_samples, N_matches); ++i) {
    SimpleMatch &s0 = samples[i][0];
    if (s0 != matches[order[n]]) ++n;
    std::stringstream sample_ss;
    sample_ss << samples[i];
    LOG(INFO) << "i: " << i << " sample: " << sample_ss.str() << " n: " << n;
    EXPECT_EQ(s0, matches[order[n]]);
  }

}  // SCENARIO
