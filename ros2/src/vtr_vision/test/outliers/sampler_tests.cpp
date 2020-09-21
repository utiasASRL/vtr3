// Internal
#include <asrl/vision/outliers.hpp>
#include <asrl/vision/TypeHelpers.hpp>

// External
#include "catch.hpp"
#include <random>
#include <chrono>
#include <cmath>

namespace av = asrl::vision;

SCENARIO("Test progressive ransac" ) {
  GIVEN("A bunch of matches") {

    // Settings
    unsigned N_matches = 10; // # of matches
    unsigned N_progressive = 100; // max # of progressive samples
    unsigned N_samples = 20; // # of samples
    unsigned m = 3; // pts per sample

    // Create the match pairs (just 1-1)
    av::SimpleMatches matches;
    matches.reserve(N_matches);
    for (unsigned int i = 0; i < N_matches; ++i)
      matches.push_back(av::SimpleMatch(i,i));

    // Create ordering (reverse)
    std::vector<unsigned> order(N_matches);
    for (unsigned i=0; i<N_matches; ++i)
      order[i] = N_matches-i-1;

    WHEN("When we sample") {

      // Create the sampler
      auto sampler = std::make_shared<av::ProgressiveSampler>(N_progressive);
      sampler->setInputMatches(&matches);
      sampler->setMatchOrder(&order);

      // Collect samples
      std::vector<av::SimpleMatches> samples(N_samples);
      for (unsigned i=0; i<N_samples; ++i) {
        REQUIRE(sampler->getSample(m, &samples[i],1000) == true);
      }

      THEN("We should have samples favouring the last points") {
        unsigned n = m-2; // Start 1 earlier, allowed to increase
        for (unsigned i=0; i<std::min(N_samples,N_matches); ++i) {
          av::SimpleMatch & s0 = samples[i][0];
          if (s0 != matches[order[n]]) ++n;
          std::stringstream sample_ss; sample_ss << samples[i];
          INFO("i: " << i << " sample: " << sample_ss.str() << " n: " << n);
          REQUIRE(s0 == matches[order[n]]);
        }
      } // THEN
    } // WHEN
  } // GIVEN
} // SCENARIO
