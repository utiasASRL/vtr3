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
 * \file test_message.cpp
 * \brief Message and LockableMessage class tests
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gmock/gmock.h>

#include <chrono>
#include <thread>

#include "rcpputils/filesystem_helper.hpp"

#include <vtr_logging/logging_init.hpp>
#include <vtr_storage/stream/data_bubble.hpp>

#include "std_msgs/msg/string.hpp"

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;
using namespace vtr::storage;
using namespace vtr::logging;

using StringMsg = std_msgs::msg::String;

class TemporaryDirectoryFixture : public Test {
 public:
  TemporaryDirectoryFixture() {
    temp_dir_ = rcpputils::fs::create_temp_directory("tmp_test_dir_").string();
    // temp_dir_ = "<set to some directory to check the resulting database>";
    // (void)rcpputils::fs::create_directories(temp_dir_);
  }

  ~TemporaryDirectoryFixture() override {
    rcpputils::fs::remove_all(rcpputils::fs::path(temp_dir_));
  }

  std::string temp_dir_;
};

TEST_F(TemporaryDirectoryFixture, accessor_interaction) {
  // accessor used by the data bubble
  auto accessor =
      std::make_shared<DataStreamAccessor<StringMsg>>(temp_dir_, "test_string");

  // construct data bubble and set accessor at construction
  DataBubble<StringMsg> db(accessor);
  EXPECT_TRUE(db.hasAccessor());

  // data bubble uses weak ptr so it does not increase the use count of accessor
  EXPECT_EQ(accessor.use_count(), 1);

  // construct data bubble and set accessor afterwards
  DataBubble<StringMsg> db2;
  EXPECT_FALSE(db2.hasAccessor());
  db2.setAccessor(accessor);
  EXPECT_TRUE(db2.hasAccessor());

  // data bubble uses weak ptr so it does not increase the use count of accessor
  EXPECT_EQ(accessor.use_count(), 1);

  // construct data bubble and set accessor at construction then reset
  DataBubble<StringMsg> db3(accessor);
  EXPECT_TRUE(db3.hasAccessor());
  db3.resetAccessor();
  EXPECT_FALSE(db3.hasAccessor());

  // clear accessor
  accessor.reset();

  // data bubble has accessor should be false
  EXPECT_FALSE(db.hasAccessor());
  EXPECT_FALSE(db2.hasAccessor());
}

TEST_F(TemporaryDirectoryFixture, load_loaded_unload) {
  // accessor used by the data bubble
  auto accessor =
      std::make_shared<DataStreamAccessor<StringMsg>>(temp_dir_, "test_string");

  // construct two data bubbles, one for write the other for read
  DataBubble<StringMsg> db(accessor), db2(accessor), db3(accessor);

  // a test message
  Timestamp timestamp = 0;
  StringMsg data;
  data.data = "data";
  auto message = std::make_shared<LockableMessage<StringMsg>>(
      std::make_shared<StringMsg>(data), timestamp);

  // store into the data bubble
  EXPECT_TRUE(db.insert(message));

  // load should not succeed because the data is not on disk yet
  EXPECT_FALSE(db.load(timestamp));
  EXPECT_FALSE(db.load(timestamp, timestamp));

  // loaded should return true because the data is indeed in cache
  EXPECT_TRUE(db.loaded(timestamp));

  // unload without clear is ok, and should save data to disk and set saved flag
  EXPECT_TRUE(db.unload(false));
  EXPECT_TRUE(message->locked().get().getSaved());

  // unload with clear should give a warning due to use count of the shared ptr
  LOG(INFO) << "Expecting warning: Message with time stamp 0 has use count 2. "
               "Keep all messages in cache.";
  EXPECT_FALSE(db.unload());

  // remove external message uses and unload again should succeed
  message.reset();
  EXPECT_TRUE(db.unload());

  // now load should succeed because data is on disk
  EXPECT_TRUE(db.load(timestamp, timestamp));
  EXPECT_EQ(db.size(), (size_t)1);  // data has been reloaded.
  EXPECT_TRUE(db.loaded(timestamp));

  // load again should give a warning but still return true since data on disk
  LOG(INFO) << "Expecting warning: Message with time stamp 0 exists. Skip "
               "loading this message into cache.";
  EXPECT_TRUE(db.load(timestamp));
  LOG(INFO) << "Expecting warning: Message with time stamp 0 exists. Skip "
               "loading this message into cache.";
  EXPECT_TRUE(db.load(timestamp, timestamp));

  // can also load message into another data bubble
  EXPECT_TRUE(db2.load(timestamp));
  EXPECT_EQ(db2.size(), (size_t)1);  // data has been reloaded.
  EXPECT_TRUE(db2.loaded(timestamp));

  // update message in data bubble 2 has no affect on data bubble 1
  auto retrieved1 = db.retrieve(timestamp);
  auto retrieved2 = db2.retrieve(timestamp);
  StringMsg data2;
  data2.data = "data2";
  retrieved2->locked().get().setData(data2);
  const auto retrieved_data1 = retrieved1->locked().get().getData();
  const auto retrieved_data2 = retrieved2->locked().get().getData();
  EXPECT_EQ(retrieved_data1.data, "data");
  EXPECT_EQ(retrieved_data2.data, "data2");

  // save data from data bubble 2 to disk then unload should update the database
  // so if we reload it from a new bubble the message should be updated.
  retrieved2.reset();
  db2.unload();
  auto retrieved3 = db3.retrieve(timestamp);
  const auto retrieved_data3 = retrieved3->locked().get().getData();
  EXPECT_EQ(retrieved_data3.data, "data2");
}

TEST_F(TemporaryDirectoryFixture, shared_ptr_to_message_concurrency) {
  // accessor used by the data bubble
  auto accessor =
      std::make_shared<DataStreamAccessor<StringMsg>>(temp_dir_, "test_string");

  // construct two data bubbles, one for write the other for read
  DataBubble<StringMsg> db(accessor);

  // a test message
  Timestamp timestamp = 0;
  StringMsg data;
  data.data = "data";
  auto message = std::make_shared<LockableMessage<StringMsg>>(
      std::make_shared<StringMsg>(data), timestamp);

  // a thread that makes some changes to the message
  std::thread th([message]() {
    // update the message
    {
      const auto locked = message->locked();
      LOG(INFO) << "Updating the message.";
      auto& message_ref = locked.get();
      // data bubble won't be able to write this message because we have
      // acquired the change lock.
      std::this_thread::sleep_for(2s);
      // make some updates to the data
      StringMsg data;
      data.data = "data updated";
      message_ref.setData(data);
      EXPECT_FALSE(message_ref.getSaved());
      LOG(INFO) << "Updating the message - done.";
    }
    // wait for the data to be saved.
    std::this_thread::sleep_for(1s);
    // check that the message has been updated
    {
      const auto locked = message->locked();
      auto& message_ref = locked.get();
      EXPECT_TRUE(message_ref.getSaved());
    }
    // wait to check if data bubble knows the correct use count
    std::this_thread::sleep_for(1s);
  });

  // store into the data bubble
  EXPECT_TRUE(db.insert(message));
  std::this_thread::sleep_for(1s);
  message.reset();

  LOG(INFO) << "Saving data bubble messages to disk.";
  LOG(INFO) << "Expecting warning: Message with time stamp 0 has use count 2. "
               "Keep all messages in cache.";
  EXPECT_FALSE(db.unload());
  LOG(INFO) << "Saving data bubble messages to disk - done.";

  // the update thread has not finished yet at this time - it checks saved
  LOG(INFO) << "Expecting warning: Message with time stamp 0 has use count 2. "
               "Keep all messages in cache.";
  EXPECT_FALSE(db.unload());

  // wait until the thread finishes using the message and destroy it.
  std::this_thread::sleep_for(2s);
  EXPECT_TRUE(db.unload());  // no warning this time

  th.join();

  DataBubble<StringMsg> db2(accessor);
  auto retrieved = db2.retrieve(timestamp);
  const auto locked = retrieved->locked();
  const auto retrieved_data = locked.get().getData();
  EXPECT_EQ(retrieved_data.data, "data updated");
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
