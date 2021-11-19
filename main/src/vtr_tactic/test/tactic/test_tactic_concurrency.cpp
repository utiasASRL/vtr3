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
 * \file test_tactic_concurrency.cpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include <chrono>

#include "vtr_logging/logging_init.hpp"
#include "vtr_tactic/tactic_v2.hpp"

using namespace ::testing;
using namespace vtr;
using namespace vtr::logging;
using namespace vtr::tactic;

class TestTactic : public PipelineInterface {
 public:
  TestTactic(const Graph::Ptr& graph, const size_t& num_async_threads,
             const size_t& async_queue_size, const int& d1 = 0,
             const int& d2 = 0, const int& d3 = 0)
      : PipelineInterface(graph, num_async_threads, async_queue_size),
        preprocess_delay_(d1),
        odometry_mapping_delay_(d2),
        localization_delay_(d3) {}

  ~TestTactic() override {
    join();
    LOG(INFO) << "TestTactic destructor done - thread joined";
  }

 private:
  /** \brief Performs the actual preprocessing task */
  bool preprocess_(const QueryCache::Ptr& qdata) override {
    std::this_thread::sleep_for(std::chrono::milliseconds(preprocess_delay_));
    return true;
  }
  /** \brief Performs the actual odometry mapping task */
  bool runOdometryMapping_(const QueryCache::Ptr& qdata) override {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(odometry_mapping_delay_));
    return true;
  }
  /** \brief Performs the actual localization task */
  bool runLocalization_(const QueryCache::Ptr& qdata) override {
    std::this_thread::sleep_for(std::chrono::milliseconds(localization_delay_));
    return true;
  }

  int preprocess_delay_;
  int odometry_mapping_delay_;
  int localization_delay_;
};

TEST(TacticConcurrency, tactic_concurrency) {
  {
    LOG(INFO) << "Data input slowly, with balanced thread usage";
    TestTactic tactic(nullptr, 2, size_t(-1), 100, 100, 100);
    for (size_t i = 0; i < 4; ++i) {
      auto qdata = std::make_shared<QueryCache>();
      qdata->stamp.emplace(i);
      tactic.input(qdata);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  {
    LOG(INFO) << "Slow preprocessing";
    TestTactic tactic(nullptr, 2, size_t(-1), 500, 100, 100);
    for (size_t i = 0; i < 4; ++i) {
      auto qdata = std::make_shared<QueryCache>();
      qdata->stamp.emplace(i);
      tactic.input(qdata);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  {
    LOG(INFO) << "Slow odometry and mapping";
    TestTactic tactic(nullptr, 2, size_t(-1), 100, 500, 100);
    for (size_t i = 0; i < 4; ++i) {
      auto qdata = std::make_shared<QueryCache>();
      qdata->stamp.emplace(i);
      tactic.input(qdata);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  {
    LOG(INFO) << "Slow localization";
    TestTactic tactic(nullptr, 2, size_t(-1), 100, 100, 500);
    for (size_t i = 0; i < 4; ++i) {
      auto qdata = std::make_shared<QueryCache>();
      qdata->stamp.emplace(i);
      tactic.input(qdata);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

int main(int argc, char** argv) {
  configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}