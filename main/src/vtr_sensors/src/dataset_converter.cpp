#include <vtr_messages/msg/rig_images.hpp>
#include <vtr_messages/msg/rig_calibration.hpp>
#include <vtr_storage/data_stream_writer.hpp>

#include <opencv2/opencv.hpp>
#include <filesystem>
#include <c++/8/fstream>

namespace fs = std::filesystem;
using RigImages = vtr_messages::msg::RigImages;
using RigCalibration = vtr_messages::msg::RigCalibration;

/// @brief Load .png images and save to rosbag2 format
int main(int argc, char *argv[]) {

  printf("Starting DataConverter!\n");

  std::string path_name = argv[argc-1];
  std::cout << path_name << "\n";

  // float x_extr = -0.239946;
  // std::vector<double> k_mat = {388.425, 0.0, 253.502, 0.0, 388.425, 196.822, 0.0, 0.0, 1.0};
  float x_extr = -0.239965;
  std::vector<double> k_mat = {387.777, 0.0, 257.446, 0.0, 387.777, 197.718, 0.0, 0.0, 1.0}; // in-the-dark
  
  for (int run=0; run < 650; run++) {

    std::string stream_name{"front_xb3"};
    std::string calibration_name{"calibration"};
    std::stringstream data_dir;
    std::stringstream stamps_path;
    std::stringstream bag_dir;
    data_dir << "/home/asrl/research/data/vtr/vtr2/" << path_name << "/run_" << std::setfill('0') << std::setw(6) << run << "/images";
    stamps_path << "/home/asrl/research/data/vtr/vtr2/" << path_name << "/run_" << std::setfill('0') << std::setw(6) << run << "/timestamps_images.txt";
    bag_dir << "/home/asrl/ASRL/data/" << path_name << "/run_" << std::setfill('0') << std::setw(6) << run;   

    std::cout << data_dir.str() << "\n";
    std::cout << fs::exists(fs::path(data_dir.str())) << "\n";

    if (fs::exists(fs::path(data_dir.str()))) {

      // std::cout << "Processing from " << data_dir.str() << " to " << bag_dir.str();

      //std::string data_dir{"/home/asrl/research/data/vtr/vtr2/path_dome_lawn2/run_000000/images"};
      // std::string stamps_path{"/home/asrl/research/data/vtr/vtr2/path_dome_lawn2/run_000000/timestamps_images.txt"};
      // std::string bag_dir{"/home/asrl/research/data/vtr/vtr3/path_dome_lawn2/run_000000"};   

      // grab timestamps from csv
      std::vector<uint64> stamp_vec;
      std::string stamp_str;
      std::ifstream ss(stamps_path.str());
      while (std::getline(ss, stamp_str)) {
        uint64 stamp = std::stoull(stamp_str);
        stamp_vec.push_back(stamp);
      }

      vtr::storage::DataStreamWriter<RigImages> writer(bag_dir.str(), stream_name);

      int img_num = 0;
      std::stringstream left_ss;
      std::stringstream right_ss;
      left_ss << data_dir.str() << "/left/" << std::setfill('0') << std::setw(6) << img_num << ".png";
      right_ss << data_dir.str() << "/right/" << std::setfill('0') << std::setw(6) << img_num << ".png";

      // iterate through all images in folder
      for (; fs::exists(fs::path(left_ss.str())) && fs::exists(fs::path(right_ss.str())); ++img_num) {

        vtr_messages::msg::RigImages sensor_message;
        sensor_message.vtr_header.sensor_time_stamp.nanoseconds_since_epoch = stamp_vec[img_num];
        vtr_messages::msg::ChannelImages chan_im;
        for (int i = 0; i < 2; ++i){
          vtr_messages::msg::Image cam_im;

          // get image from png
          cv::Mat im;
          std::string cam_name;
          if (i == 0){
            im = cv::imread(left_ss.str(), cv::IMREAD_COLOR);
            cam_name = "left";
          } else {
            im = cv::imread(right_ss.str(), cv::IMREAD_COLOR);
            cam_name = "right";
          }

          cam_im.name = cam_name;
          cam_im.height = im.rows;
          cam_im.width = im.cols;
          cam_im.depth = 3;
          cam_im.encoding = "bgr8";
          cam_im.is_bigendian = false;
          cam_im.stamp.nanoseconds_since_epoch = stamp_vec[img_num];
          cam_im.step = 512; //im.step[0];
          cam_im.data.resize(cam_im.height * cam_im.width * 3);
          cam_im.data.assign(im.data, im.data+im.total()*im.channels());

          chan_im.cameras.push_back(cam_im);
        }
        sensor_message.channels.push_back(chan_im);

        // write to rosbag
        writer.write(sensor_message);

        // update image paths
        left_ss.str(std::string());
        left_ss.clear();
        left_ss << data_dir.str() << "/left/" << std::setfill('0') << std::setw(6) << img_num + 1 << ".png";
        right_ss.str(std::string());
        right_ss.clear();
        right_ss << data_dir.str() << "/right/" << std::setfill('0') << std::setw(6) << img_num + 1 << ".png";
      }

      // Calibration message
      vtr::storage::DataStreamWriter<RigCalibration> calibration_writer(bag_dir.str(), calibration_name);
        
      vtr_messages::msg::RigCalibration rig_calibration_message;
      vtr_messages::msg::CameraCalibration camera_calibration_message;
      vtr_messages::msg::Transform transform_message_left;
      vtr_messages::msg::Transform transform_message_right;
      vtr_messages::msg::Vec3 vec3_message_left;
      vtr_messages::msg::Vec3 vec3_message_right;
      vtr_messages::msg::AxisAngle axis_angle_message;
      
      vec3_message_left.x = 0.0;
      vec3_message_left.y = 0.0;
      vec3_message_left.z = 0.0;

      vec3_message_right.x = x_extr;
      vec3_message_right.y = 0.0;
      vec3_message_right.z = 0.0;

      axis_angle_message.x = 0.0;
      axis_angle_message.y = 0.0;
      axis_angle_message.z = 0.0;

      transform_message_left.translation = vec3_message_left;
      transform_message_left.orientation = axis_angle_message;

      transform_message_right.translation = vec3_message_right;
      transform_message_right.orientation = axis_angle_message;

      camera_calibration_message.k_mat = k_mat;

      rig_calibration_message.intrinsics.push_back(camera_calibration_message); // Same k_mat for left and right camera
      rig_calibration_message.intrinsics.push_back(camera_calibration_message);
      rig_calibration_message.extrinsics.push_back(transform_message_left);
      rig_calibration_message.extrinsics.push_back(transform_message_right);
      rig_calibration_message.rectified = true;

      // write to rosbag
      calibration_writer.write(rig_calibration_message);
    }

    data_dir.str(std::string());
    data_dir.clear();
    stamps_path.str(std::string());
    stamps_path.clear();
    bag_dir.str(std::string());
    bag_dir.clear();

  }

  return 0;
}