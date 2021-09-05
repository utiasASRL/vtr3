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
 * \file read_write_calibration_test.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include <filesystem>

#include <vtr_storage/stream/data_stream_reader.hpp>
#include <vtr_storage/stream/data_stream_writer.hpp>
#include <vtr_storage/stream/message.hpp>

#include "test_msgs/msg/basic_types.hpp"
using TestMsg = test_msgs::msg::BasicTypes;
using RigCalibrationMsg = vtr_messages::msg::RigCalibration;
using CameraCalibrationMsg = vtr_messages::msg::CameraCalibration;

namespace fs = std::filesystem;

using namespace vtr::storage;

// sample code showing how to use the data streams
TEST(VTRStorage, readWriteCalibration) {
  fs::path working_dir{fs::temp_directory_path() / "vtr_storage_test"};
  fs::remove_all(working_dir);  // make sure the directory is empty.
  std::string stream_name = "test_stream";

  TestMsg test_msg;

  // write data
  DataStreamWriter<TestMsg> writer(working_dir, stream_name);
  for (int i = 1; i <= 10; i++) {
    test_msg.float64_value = i * 10;
    writer.write(VTRMessage(test_msg));
  }

  // attempt to read calibration, should fail
  DataStreamReader<TestMsg, RigCalibrationMsg> reader(working_dir, stream_name);
  EXPECT_THROW(reader.fetchCalibration(), NoBagExistsException);

  // write a calibration msg
  RigCalibrationMsg calibration_msg;
  CameraCalibrationMsg intrinsics;
  intrinsics.k_mat = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  calibration_msg.intrinsics.push_back(intrinsics);
  DataStreamWriterCalibration calibration_writer(working_dir);
  calibration_writer.write(calibration_msg);

  // read calibration and data
  auto calibration =
      reader.fetchCalibration()->template get<RigCalibrationMsg>();
  EXPECT_EQ(calibration, calibration_msg);

  auto bag_message_vector = reader.readAtIndexRange(1, 9);
  int count = 1;
  for (auto message : *bag_message_vector) {
    test_msg = message->template get<TestMsg>();
    EXPECT_EQ(test_msg.float64_value, count * 10);
    ++count;
  }

  // test reading calibration with no calibration type specified
  // should assert() and crash
  DataStreamReader<TestMsg> reader2(working_dir, stream_name);
  ASSERT_DEATH(reader2.fetchCalibration(), "");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
