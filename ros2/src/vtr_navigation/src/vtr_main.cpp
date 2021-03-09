#include <filesystem>

#include "rclcpp/rclcpp.hpp"

#include <vtr_common/timing/time_utils.hpp>
#include <vtr_common/utils/filesystem.hpp>
#include <vtr_logging/logging_init.hpp>
#include <vtr_navigation/navigator.hpp>

namespace fs = std::filesystem;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::navigation;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("vtr");

  // Log into a subfolder of the data directory (if requested to log)
  std::string log_filename;
  auto to_file = node->declare_parameter<bool>("log_to_file", false);
  if (to_file) {
    auto data_dir_str = node->declare_parameter<std::string>("data_dir", "");
    auto log_name = timing::toIsoFilename(timing::clock::now());
    fs::path data_dir{utils::expand_user(data_dir_str)};
    log_filename = data_dir / "logs" / (log_name + ".log");
  }
  configureLogging(log_filename);
  LOG_IF(to_file, INFO) << "Logging to: " << log_filename;
  LOG_IF(!to_file, WARNING) << "NOT LOGGING TO A FILE.";

  LOG(INFO) << "Starting Navigation Node, beep bop boop";
  Navigator nav{node};

  // Wait for shutdown
  rclcpp::spin(node);
  rclcpp::shutdown();
}
