// #define _GLIBCXX_USE_NANOSLEEP 1

#include <experimental/filesystem>
#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <vtr/testing/module_vo.h>

#include <robochunk_msgs/MessageBase.pb.h>
#include <robochunk_msgs/XB3CalibrationRequest.pb.h>
#include <robochunk_msgs/XB3CalibrationResponse.pb.h>
#include <robochunk/base/DataStream.hpp>
#include <robochunk/util/fileUtils.hpp>

#include <asrl/common/logging.hpp>  // easylogging++
#include <asrl/common/timing/SimpleTimer.hpp>
#include <asrl/vision/messages/bridge.hpp>

namespace fs = std::experimental::filesystem;

// ** FOLLOWING LINE SHOULD BE USED ONCE AND ONLY ONCE IN WHOLE APPLICATION **
// ** THE BEST PLACE TO PUT THIS LINE IS IN main.cpp RIGHT AFTER INCLUDING
// easylogging++.h **
INITIALIZE_EASYLOGGINGPP

// \todo Figure out why this function must be placed here.
void placeholder(const robochunk::sensor_msgs::RigCalibration &robo_calib) {
  std::cout
      << "This function has to be here to make the code run. Even if it "
         "is never used????? The code below is also needed for some reasone "
         "(maybe to prevent the optimizer from optimizing this function?)";
  asrl::vision::RigCalibration calibration;
  // calibration.rectified = robo_calib.rectified();
  auto num_cameras = robo_calib.intrinsics().size();
  for (int idx = 0; idx < num_cameras; ++idx) {
    calibration.intrinsics.push_back(
        asrl::messages::copyIntrinsics(robo_calib.intrinsics().Get(idx)));
    calibration.extrinsics.push_back(
        asrl::messages::copyExtrinsics(robo_calib.extrinsics().Get(idx)));
  }
}

int main(int argc, char **argv) {
  LOG(INFO) << "Starting Module VO, beep beep beep";

  // // enable parallelisation
  // Eigen::initParallel(); // This is no longer needed in Eigen3?

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

  ModuleVO vo(nh, results_dir);

  robochunk::msgs::RobochunkMessage calibration_msg;

  LOG(INFO) << "Fetching calibration..." << std::endl;
  // Check out the calibration
  if (stereo_stream.fetchCalibration(calibration_msg) == true) {
    LOG(INFO) << "Calibration fetched..." << std::endl;
    std::shared_ptr<asrl::vision::RigCalibration> rig_calib = nullptr;

    LOG(INFO) << "Trying to extract a sensor_msgs::RigCalibration...";
    auto rig_calibration =
        calibration_msg
            .extractSharedPayload<robochunk::sensor_msgs::RigCalibration>();
    if (rig_calibration == nullptr) {
      LOG(WARNING)
          << "Trying to extract a sensor_msgs::RigCalibration failed, so I'm "
             "going to try to extract a sensor_msgs::XB3CalibrationResponse...";
      auto xb3_calibration = calibration_msg.extractSharedPayload<
          robochunk::sensor_msgs::XB3CalibrationResponse>();
      if (xb3_calibration == nullptr) {
        LOG(ERROR) << "Trying to extract a sensor_msgs::XB3CalibrationResponse "
                      "failed. Calibration extraction failed!!";
      } else {
        LOG(INFO)
            << "Successfully extracted a sensor_msgs::XB3CalibrationResponse.";
        rig_calib = std::make_shared<asrl::vision::RigCalibration>(
            asrl::messages::copyCalibration(*xb3_calibration));
      }
    } else {
      LOG(INFO) << "Successfully extracted a sensor_msgs::RigCalibration.";
      rig_calib = std::make_shared<asrl::vision::RigCalibration>(
          asrl::messages::copyCalibration(*rig_calibration));
    }

    if (rig_calib != nullptr) {
      LOG(INFO) << "Received camera calibration!";
      vo.setCalibration(rig_calib);
    } else {
      LOG(ERROR) << "ERROR: intrinsic params is not the correct type: (actual: "
                 << calibration_msg.header().type_name().c_str() << ")";
      return -1;
    }
  } else {
    LOG(ERROR) << "ERROR: Could not read calibration message!";
  }

  // Seek to an absolute index
  stereo_stream.seek(static_cast<uint32_t>(start_index));

  // Get the first message
  bool continue_stream = true;
  robochunk::msgs::RobochunkMessage data_msg;
  continue_stream &= stereo_stream.next(data_msg);
  int idx = 0;
  asrl::common::timing::SimpleTimer timer;  // unused
  while (continue_stream == true && idx + start_index < stop_index &&
         ros::ok()) {
    auto rig_images =
        data_msg.extractSharedPayload<robochunk::sensor_msgs::RigImages>();
    if (rig_images != nullptr) {
      LOG(INFO) << "Processing the " << idx << "th image.";
      vo.processImageData(rig_images, data_msg.header().sensor_time_stamp());
    } else
      LOG(ERROR) << "Data is nullptr!";
    data_msg.Clear();
    continue_stream &= stereo_stream.next(data_msg);
    idx++;
  }
  LOG(INFO) << "Time to exit!";
}
