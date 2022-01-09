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
 * \file test_query_buffer.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include <chrono>

#include "vtr_logging/logging_init.hpp"
#include "vtr_tactic/tactic.hpp"

using namespace ::testing;
using namespace vtr;
using namespace vtr::logging;
using namespace vtr::tactic;

TEST(TacticQueryBuffer, query_buffer_blocking_pop_and_push) {
  size_t buffer_size = 5;
  QueryBuffer<size_t> buffer(buffer_size);

  auto producer = [&]() {
    // discardable, non-discardable
    for (size_t i = 0; i < 10; ++i) {
      LOG(INFO) << "Producing: " << i << ", discardable: " << std::boolalpha
                << (bool)(i % 2);
      buffer.push(i, i % 2);
    }
    // blocking push
    for (size_t i = 0; i < 3; ++i) {
      LOG(INFO) << "Producing: " << i << ", discardable: " << std::boolalpha
                << false;
      buffer.push(i, false);
    }
    for (size_t i = 0; i < 3; ++i) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      LOG(INFO) << "Producing: " << i << ", discardable: " << std::boolalpha
                << false;
      buffer.push(i, false);
    }
  };

  auto consumer = [&]() {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    for (size_t i = 0; i < 5; ++i) {
      const auto& val = buffer.pop();
      LOG(INFO) << "Consuming: " << val;
      EXPECT_EQ(val, i * 2);
    }
    for (size_t i = 0; i < 3; ++i) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      const auto& val = buffer.pop();
      LOG(INFO) << "Consuming: " << val;
      EXPECT_EQ(val, i);
    }
    // blocking pop
    for (size_t i = 0; i < 3; ++i) {
      const auto& val = buffer.pop();
      LOG(INFO) << "Consuming: " << val;
      EXPECT_EQ(val, i);
    }
  };

  std::thread producer_thread(producer);
  std::thread consumer_thread(consumer);

  buffer.wait(buffer_size);  // wait for buffer to be full
  LOG(INFO) << "Buffer is full";
  buffer.wait();  // wait until the buffer is empty
  LOG(INFO) << "Buffer is empty";

  producer_thread.join();
  consumer_thread.join();
}

TEST(TacticQueryBuffer, query_buffer_require_immediate_pop_slow_pop_discard) {
  QueryBuffer<size_t> buffer(0);

  auto producer = [&]() {
    // discardable, non-discardable
    for (size_t i = 0; i < 5; ++i) {
      LOG(INFO) << "Producing: " << i << ", discardable: true";
      const auto discarded = buffer.push(i, true);
      LOG(INFO) << "Message discarded: " << std::boolalpha << discarded;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    for (size_t i = 0; i < 5; ++i) {
      LOG(INFO) << "Producing: " << i << ", discardable: true";
      const auto discarded = buffer.push(i, true);
      LOG(INFO) << "Message discarded: " << std::boolalpha << discarded;
    }
  };

  auto consumer = [&]() {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const auto& val = buffer.pop();
    LOG(INFO) << "Consuming: " << val;
  };

  std::thread producer_thread(producer);
  std::thread consumer_thread(consumer);

  buffer.wait();  // wait until the buffer is empty
  LOG(INFO) << "Buffer is empty";

  producer_thread.join();
  consumer_thread.join();
}

TEST(TacticQueryBuffer,
     query_buffer_require_immediate_pop_slow_pop_nondiscard) {
  QueryBuffer<size_t> buffer(0);

  auto producer = [&]() {
    // discardable, non-discardable
    for (size_t i = 0; i < 1000; ++i) {
      LOG(INFO) << "Producing: " << i << ", discardable: false";
      const auto discarded = buffer.push(i, false);
      LOG(INFO) << "Message discarded: " << std::boolalpha << discarded;
    }
  };

  auto consumer = [&]() {
    for (size_t i = 0; i < 1000; ++i) {
      const auto& val = buffer.pop();
      LOG(INFO) << "Consuming: " << val;
    }
  };

  std::thread producer_thread(producer);
  std::thread consumer_thread(consumer);

  buffer.wait();  // wait until the buffer is empty
  LOG(INFO) << "Buffer is empty";

  producer_thread.join();
  consumer_thread.join();
}

TEST(TacticQueryBuffer, query_buffer_require_immediate_pop_fast_pop) {
  QueryBuffer<size_t> buffer(0);

  auto producer1 = [&]() {
    // discardable, non-discardable
    for (size_t i = 0; i < 10; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      LOG(INFO) << "Producing: " << i << ", discardable: " << std::boolalpha
                << (bool)(i % 2);
      const auto discarded = buffer.push(i, i % 2);
      LOG(INFO) << "Message discarded in p1: " << std::boolalpha << discarded;
    }
  };

  auto consumer1 = [&]() {
    for (size_t i = 0; i < 10; ++i) {
      const auto& val = buffer.pop();
      LOG(INFO) << "Consuming in c1: " << val;
    }
  };

  std::thread producer_thread1(producer1);
  std::thread consumer_thread1(consumer1);

  buffer.wait();  // wait until the buffer is empty
  LOG(INFO) << "Buffer is empty";

  producer_thread1.join();
  consumer_thread1.join();
}

int main(int argc, char** argv) {
  configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}