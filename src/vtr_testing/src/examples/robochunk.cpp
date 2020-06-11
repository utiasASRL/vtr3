// #define _GLIBCXX_USE_NANOSLEEP 1

#include <filesystem>
#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <robochunk_msgs/MessageBase.pb.h>
#include <robochunk_msgs/XB3CalibrationRequest.pb.h>
#include <robochunk_msgs/XB3CalibrationResponse.pb.h>
#include <robochunk/base/DataStream.hpp>
#include <robochunk/util/fileUtils.hpp>

#include <asrl/common/logging.hpp>  // easylogging++
// #include <asrl/common/timing/SimpleTimer.hpp>
// #include <asrl/vision/messages/bridge.hpp>

namespace fs = std::filesystem;

// ** FOLLOWING LINE SHOULD BE USED ONCE AND ONLY ONCE IN WHOLE APPLICATION **
// ** THE BEST PLACE TO PUT THIS LINE IS IN main.cpp RIGHT AFTER INCLUDING
// easylogging++.h **
INITIALIZE_EASYLOGGINGPP

int main(int argc, char **argv) {
  LOG(INFO) << "Starting RoboChunk, beep beep beep";

  // ros publishers for visualisations
  ros::init(argc, argv, "module_vo");
  ros::NodeHandle nh("~");
  std::string prefix("module_vo/");

  std::string data_dir_str, results_dir_str, sim_run_str, stream_name;
  nh.param<std::string>("input_data_dir", data_dir_str, "/path/to/data/");
  nh.param<std::string>("sim_run", sim_run_str, "/sim_run/");
  nh.param<std::string>("stream_name", stream_name, "/data_stream/");
  nh.param<std::string>("results_dir", results_dir_str, "/path/to/results/");

  auto data_dir = fs::path(data_dir_str);
  auto results_dir = fs::path(results_dir_str);
  auto sim_run = fs::path(sim_run_str);

  int start_index;
  int stop_index;
  nh.param<int>("start_index", start_index, 1);
  nh.param<int>("stop_index", stop_index, 20000);

  robochunk::base::ChunkStream stereo_stream(data_dir / sim_run, stream_name);
}
