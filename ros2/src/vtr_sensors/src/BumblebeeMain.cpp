
#include <vtr_sensors/BumblebeeXb3.hpp>

#include <filesystem>

namespace fs = std::filesystem;


//int main() {
int main(int argc, char **argv) {


  rclcpp::init(argc, argv);
/*  auto node = rclcpp::Node::make_shared("talker");*/

  vtr::sensors::xb3::Xb3Configuration config;

#if 0   //todo: load config from yaml
  std::string config_dir("");
#if 0
  struct stat st;
  if (stat(std::string(SENSORS_INSTALL_DIR).c_str(),&st) == 0) {
    if(st.st_mode & S_IFDIR != 0)
      config_dir = std::string(SENSORS_INSTALL_DIR);
  } else if(stat(std::string(SENSORS_DEVEL_DIR).c_str(),&st) == 0) {
    if(st.st_mode & S_IFDIR != 0)
      config_dir = std::string(SENSORS_DEVEL_DIR);
  }
#endif
  std::string config_path(config_dir + "/configuration.xml");

  if (argc >= 2) {
    try {
      std::string tmp(argv[1]);
      if (fs::exists(tmp)) {
        config_path = tmp;
      } else {
#if 0
        LOG(WARNING) << "No config file at path " << tmp <<"; using global default.";
#endif
      }
    } catch (...) {
#if 0
      LOG(ERROR) << "Something went wrong parsing command line args...";
#endif
      config_path = config_dir + "/configuration.xml";
    }
  }

  pugi::xml_document doc;
  pugi::xml_parse_result xmlLocation;

  try {
    xmlLocation = doc.load_file(
        config_path.c_str(), pugi::parse_eol);

    // PugiXML general config Node.
    pugi::xml_node general_p = doc.child("config").child("general");
    config.data_mode = general_p.attribute("data_mode").value();
    config.camera_model = general_p.attribute("camera_model").value();
    config.camera_name = general_p.attribute("camera_name").value();
    config.packet_multiplier = general_p.attribute("packet_multiplier").as_int();

    config.rectified_image_size = cv::Size(
        general_p.attribute("rectified_image_size_width").as_int(),
        general_p.attribute("rectified_image_size_height").as_int());
    config.output_gray = general_p.attribute("output_gray").as_bool();

    config.data_directory = general_p.attribute("data_directory").value();
    config.stream_name = general_p.attribute("stream_name").value();
    config.start_time = std::stol(general_p.attribute("start_time").value());
    config.end_time = std::stol(general_p.attribute("end_time").value());
    config.start_idx = general_p.attribute("start_idx").as_uint();
    config.end_idx = general_p.attribute("end_idx").as_uint();

    std::string playback_mode = general_p.attribute("playback_mode").value();
    if (playback_mode == "RATE") {
      config.playback_mode = robochunk::comms::SensorConfiguration::PlaybackMode::RATE;
    }
    else if (playback_mode == "AFAP") {
      config.playback_mode = robochunk::comms::SensorConfiguration::PlaybackMode::AFAP;
    }

    config.playback_speed = general_p.attribute("playback_speed").as_double();
    config.sim_time = general_p.attribute("sim_time").as_bool();
    config.fudge_time = general_p.attribute("fudge_time").as_bool();

    // Published data comms info
    pugi::xml_node data_comms_p = doc.child("config").child("data_comms");
    config.data_comms.topic = data_comms_p.attribute("topic").value();
    config.data_comms.address = data_comms_p.attribute("address").value();
    config.data_comms.port = data_comms_p.attribute("port").as_uint();

    // Stereo Images
    pugi::xml_node clock_p = doc.child("config").child("clock");
    config.clock.topic = clock_p.attribute("topic").value();
    config.clock.address = clock_p.attribute("address").value();
    config.clock.port = clock_p.attribute("port").as_uint();

    // Calibration
    pugi::xml_node calibration_server_p =
        doc.child("config").child("calibration_server");
    config.calibration_server.topic =
        calibration_server_p.attribute("topic").value();
    config.calibration_server.address =
        calibration_server_p.attribute("address").value();
    config.calibration_server.port =
        calibration_server_p.attribute("port").as_uint();

    // Display
    pugi::xml_node display_p =
        doc.child("config").child("display");
    config.show_rectified_images =
        display_p.attribute("show_rectified_images").as_bool();
    config.show_raw_images =
        display_p.attribute("show_raw_images").as_bool();
  } catch (...) {
#if 0
    LOG(ERROR) << "Could not load config file!";
#endif
    return -2;
  }
#else
  config.camera_model = "XB3";
  config.camera_name = "front_xb3";
  config.rectified_image_size = cv::Size(512,384);
  config.output_gray = false;
  config.show_raw_images = true;
  config.show_rectified_images = true;
  config.packet_multiplier = 0;

#endif

  try {
    vtr::sensors::xb3::BumblebeeXb3 cameraDriver(config);
    return cameraDriver.run();
  } catch (const std::exception& e) {
#if 0
    LOG(ERROR) << "Unhandled exception in camera driver: " << e.what();
#endif
    return -3;
  }
}