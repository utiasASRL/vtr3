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
#include <filesystem>
#include <thread>

#include <boost/thread.hpp>  // std::lock that takes iterator input

#include <vtr_storage/stream/message.hpp>

using namespace std::chrono_literals;
using namespace vtr::storage;

TEST(TestMessage, constructing_getting_setting_message) {
  std::string data0{"data to be saved 0"};
  std::string data1{"data to be saved 1"};
  std::string data2{"data to be saved 2"};

  /// construct the message with data only
  {
    const auto data_ptr = std::make_shared<std::string>(data0);
    Message message{data_ptr};
    // message information (no index given implies not saved)
    EXPECT_EQ(message.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(message.getIndex(), NO_INDEX_VALUE);
    EXPECT_EQ(message.getSaved(), false);

    // get data returns a copy of the data
    const auto saved_data = message.getData<std::string>();
    EXPECT_EQ(data0, saved_data);
    // get data ptr returns the shared pointer to the stored data
    const auto saved_data_ptr = message.getDataPtr<std::string>();
    EXPECT_EQ(data0, *saved_data_ptr);

    // set data will update the data
    message.setData<std::string>(data1);
    // get data returns a copy of the data
    const auto saved_data1 = message.getData<std::string>();
    EXPECT_EQ(data1, saved_data1);
    EXPECT_EQ(data1, *saved_data_ptr);  // shared pointer should be updated auto

    // get data of a different type unfortunately does not throw, but is
    // undefined behavior!
    // EXPECT_NO_THROW(message.getData<bool>(), std::runtime_error);

    // set index -> saved
    message.setIndex(1);
    EXPECT_EQ(message.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(message.getIndex(), 1);
    EXPECT_EQ(message.getSaved(), true);
  }

  /// construct the message with data and timestamp/index information
  {
    const auto data_ptr = std::make_shared<std::string>(data0);
    Message message{data_ptr, 100, 1};

    // message information (has >0 index implies saved)
    EXPECT_EQ(message.getTimestamp(), 100);
    EXPECT_EQ(message.getIndex(), 1);
    EXPECT_EQ(message.getSaved(), true);

    // set data -> unsaved
    message.setData<std::string>(data1);
    EXPECT_EQ(message.getSaved(), false);
    message.setSaved();  // default to true
    EXPECT_EQ(message.getSaved(), true);

    // set timestamp -> unsaved
    message.setTimestamp(200);
    EXPECT_EQ(message.getSaved(), false);
    message.setSaved();  // default to true
    EXPECT_EQ(message.getSaved(), true);

    // set index -> throw because it already has an index
    EXPECT_THROW(message.setIndex(2), std::runtime_error);

    // set to the same index is ok  -> saved
    message.setIndex(1);
    EXPECT_EQ(message.getSaved(), true);
  }
}

TEST(TestLockableMessage, constructing_getting_setting_lockable_message) {
  std::string data0{"data to be saved 0"};
  std::string data1{"data to be saved 1"};
  std::string data2{"data to be saved 2"};

  /// get a locked non-const reference to data
  {
    const auto data_ptr = std::make_shared<std::string>(data0);
    LockableMessage lockable_message{data_ptr};

    /// check locked reference
    auto& message = lockable_message.locked().get();

    // message information (no index given implies not saved)
    EXPECT_EQ(message.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(message.getIndex(), NO_INDEX_VALUE);
    EXPECT_EQ(message.getSaved(), false);

    // get data returns a copy of the data
    const auto saved_data = message.getData<std::string>();
    EXPECT_EQ(data0, saved_data);
    // get data ptr returns the shared pointer to the stored data
    const auto saved_data_ptr = message.getDataPtr<std::string>();
    EXPECT_EQ(data0, *saved_data_ptr);

    // set data will update the data
    message.setData<std::string>(data1);
    // get data returns a copy of the data
    const auto saved_data1 = message.getData<std::string>();
    EXPECT_EQ(data1, saved_data1);
    EXPECT_EQ(data1, *saved_data_ptr);  // shared pointer should be updated

    // set index -> saved
    message.setIndex(1);
    EXPECT_EQ(message.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(message.getIndex(), 1);
    EXPECT_EQ(message.getSaved(), true);
  }

  /// get a locked const reference to data
  {
    const auto data_ptr = std::make_shared<std::string>(data0);
    LockableMessage lockable_message{data_ptr};

    /// check locked reference
    const auto& message = lockable_message.locked().get();

    // message information (no index given implies not saved)
    EXPECT_EQ(message.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(message.getIndex(), NO_INDEX_VALUE);
    EXPECT_EQ(message.getSaved(), false);

    // get data returns a copy of the data
    const auto saved_data = message.getData<std::string>();
    EXPECT_EQ(data0, saved_data);
    // get data ptr returns the shared pointer to the stored data
    const auto saved_data_ptr = message.getDataPtr<std::string>();
    EXPECT_EQ(data0, *saved_data_ptr);

    // set operations are not permitted
  }

  /// get an unlocked const reference to data
  {
    const auto data_ptr = std::make_shared<std::string>(data0);
    LockableMessage lockable_message{data_ptr};

    /// check unlocked reference
    const auto& message = lockable_message.unlocked().get();

    // message information (no index given implies not saved)
    EXPECT_EQ(message.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(message.getIndex(), NO_INDEX_VALUE);
    EXPECT_EQ(message.getSaved(), false);

    // get data returns a copy of the data
    const auto saved_data = message.getData<std::string>();
    EXPECT_EQ(data0, saved_data);
    // get data ptr returns the shared pointer to the stored data
    const auto saved_data_ptr = message.getDataPtr<std::string>();
    EXPECT_EQ(data0, *saved_data_ptr);

    // set operations are not permitted
  }
}

TEST(TestLockableMessage, constructing_getting_setting_message) {
  std::string data0{"data to be saved 0"};
  std::string data1{"data to be saved 1"};
  std::string data2{"data to be saved 2"};

  /// typical deadlock scenario - NEVER ACCESS MULTIPLE MESSAGES THIS WAY!
  // {
  //   LockableMessage message0{std::make_shared<std::string>(data0)};
  //   LockableMessage message1{std::make_shared<std::string>(data1)};

  //   std::thread t1([&message0, &message1]() {
  //     const auto locked0 = message0.locked();
  //     std::cout << "message0 locked" << std::endl;
  //     std::this_thread::sleep_for(1s);
  //     const auto locked1 = message1.locked();
  //     std::cout << "message1 locked" << std::endl;
  //   });
  //   std::thread t2([&message0, &message1]() {
  //     const auto locked1 = message1.locked();
  //     std::cout << "message1 locked" << std::endl;
  //     std::this_thread::sleep_for(1s);
  //     const auto locked0 = message0.locked();
  //     std::cout << "message0 locked" << std::endl;
  //   });

  //   t1.join();
  //   t2.join();
  // }

  /// access two messages without locking both at the same time is ok
  {
    LockableMessage message0{std::make_shared<std::string>(data0)};
    LockableMessage message1{std::make_shared<std::string>(data1)};

    std::thread t1([&message0, &message1]() {
      [[maybe_unused]] const auto& locked0 = message0.locked().get();
      std::cout << "task1 message0" << std::endl;
      std::this_thread::sleep_for(1s);
      [[maybe_unused]] const auto& locked1 = message1.locked().get();
      std::cout << "task1 message1" << std::endl;
    });
    std::thread t2([&message0, &message1]() {
      [[maybe_unused]] const auto& locked1 = message1.locked().get();
      std::cout << "task2 message1" << std::endl;
      std::this_thread::sleep_for(1s);
      [[maybe_unused]] const auto& locked0 = message0.locked().get();
      std::cout << "task2 message0" << std::endl;
    });

    t1.join();
    t2.join();
  }

  /// access two messages acquiring both locks simutaneously is ok
  {
    LockableMessage message0{std::make_shared<std::string>(data0)};
    LockableMessage message1{std::make_shared<std::string>(data1)};

    std::thread t1([&message0, &message1]() {
      using LockType = std::unique_lock<std::shared_mutex>;
      // lock both simutaneously is ok
      LockType lk1(message0.mutex(), std::defer_lock);
      LockType lk2(message1.mutex(), std::defer_lock);
      std::lock(lk1, lk2);
      [[maybe_unused]] const auto locked0 = message0.unlocked();
      std::cout << "task1 message0" << std::endl;
      std::this_thread::sleep_for(1s);
      [[maybe_unused]] const auto locked1 = message1.unlocked();
      std::cout << "task1 message1" << std::endl;
    });
    // this is bad - ok here because the above thread lock both simutaneously
    std::thread t2([&message0, &message1]() {
      [[maybe_unused]] const auto locked1 = message1.locked();
      std::cout << "task2 message1" << std::endl;
      std::this_thread::sleep_for(1s);
      [[maybe_unused]] const auto locked0 = message0.locked();
      std::cout << "task2 message0" << std::endl;
    });

    t1.join();
    t2.join();
  }

  /// access two messages acquiring both locks simutaneously is ok
  {
    LockableMessage message0{std::make_shared<std::string>(data0)};
    LockableMessage message1{std::make_shared<std::string>(data1)};

    std::thread t1([&message0, &message1]() {
      using LockType = std::unique_lock<std::shared_mutex>;
      // second way of locking both simutaneously, only boost library provides
      // this locking function at the moment
      std::vector<LockType> lks;
      lks.emplace_back(message0.mutex(), std::defer_lock);
      lks.emplace_back(message1.mutex(), std::defer_lock);
      boost::lock(lks.begin(), lks.end());

      [[maybe_unused]] const auto locked0 = message0.unlocked();
      std::cout << "task1 message0" << std::endl;
      std::this_thread::sleep_for(1s);
      [[maybe_unused]] const auto locked1 = message1.unlocked();
      std::cout << "task1 message1" << std::endl;
    });
    // this is bad - ok here because the above thread lock both simutaneously
    std::thread t2([&message0, &message1]() {
      [[maybe_unused]] const auto locked1 = message1.locked();
      std::cout << "task2 message1" << std::endl;
      std::this_thread::sleep_for(1s);
      [[maybe_unused]] const auto locked0 = message0.locked();
      std::cout << "task2 message0" << std::endl;
    });

    t1.join();
    t2.join();
  }

  /// access two messages acquiring both locks simutaneously is ok
  {
    LockableMessage message0{std::make_shared<std::string>(data0)};
    LockableMessage message1{std::make_shared<std::string>(data1)};

    std::thread t1([&message0, &message1]() {
      // third way of locking both simutaneously
      std::lock(message0.mutex(), message1.mutex());

      [[maybe_unused]] const auto locked0 = message0.unlocked();
      std::cout << "task1 message0" << std::endl;
      std::this_thread::sleep_for(1s);
      [[maybe_unused]] const auto locked1 = message1.unlocked();
      std::cout << "task1 message1" << std::endl;

      // must unlock both at the end
      message0.unlock();
      message1.unlock();
    });
    // this is bad - ok here because the above thread lock both simutaneously
    std::thread t2([&message0, &message1]() {
      [[maybe_unused]] const auto locked1 = message1.locked();
      std::cout << "task2 message1" << std::endl;
      std::this_thread::sleep_for(1s);
      [[maybe_unused]] const auto locked0 = message0.locked();
      std::cout << "task2 message0" << std::endl;
    });

    t1.join();
    t2.join();
  }
}