#include <gtest/gtest.h>
#include <filesystem>

#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "vtr_storage/data_stream_reader.hpp"
#include "vtr_storage/data_stream_writer.hpp"
#include "vtr_storage/message.hpp"

#include "test_msgs/msg/basic_types.hpp"

namespace fs = std::filesystem;
using RigCalibration = vtr_messages::msg::RigCalibration;

// sample code showing how to use the data streams
TEST(VTRStorage, readWriteCalibration) {
  fs::path working_dir{fs::temp_directory_path() / "vtr_storage_test"};
  fs::remove_all(working_dir);  // make sure the directory is empty.
  std::string stream_name = "test_stream";
  using TestMsgT = test_msgs::msg::BasicTypes;
  TestMsgT test_msg;

  // write data
  vtr::storage::DataStreamWriter<TestMsgT> writer(working_dir, stream_name);
  for (int i = 1; i <= 10; i++) {
    test_msg.float64_value = i * 10;
    writer.write(vtr::storage::VTRMessage(test_msg));
  }

  // attempt to read calibration, should fail
  vtr::storage::DataStreamReader<TestMsgT, RigCalibration> reader(working_dir, stream_name);
  EXPECT_THROW(reader.fetchCalibration(), vtr::storage::NoBagExistsException);

  // write a calibration msg
  vtr_messages::msg::RigCalibration calibration_msg;
  vtr_messages::msg::CameraCalibration intrinsics;
  intrinsics.k_mat = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  calibration_msg.intrinsics.push_back(intrinsics);
  vtr::storage::DataStreamWriterCalibration calibration_writer(working_dir);
  calibration_writer.write(calibration_msg);

  // read calibration and data
  auto calibration = reader.fetchCalibration()->template get<RigCalibration>();
  EXPECT_EQ(calibration, calibration_msg);

  auto bag_message_vector = reader.readAtIndexRange(1, 9);
  int count = 1;
  for (auto message : *bag_message_vector) {
    test_msg = message->template get<TestMsgT>();
    EXPECT_EQ(test_msg.float64_value, count*10);
    ++count;
  }

  // test reading calibration with no calibration type specified
  // should assert() and crash
  vtr::storage::DataStreamReader<TestMsgT> reader2(working_dir, stream_name);
  ASSERT_DEATH(reader2.fetchCalibration(),"");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
