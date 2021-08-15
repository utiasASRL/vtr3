/**
 * \file main.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <filesystem>

#include <vtr_common/timing/time_utils.hpp>
#include <vtr_common/utils/filesystem.hpp>
#include <vtr_logging/logging_init.hpp>
#include <vtr_safety_monitor/safety_monitor/safety_monitor.hpp>

namespace fs = std::filesystem;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::safety_monitor;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("safety_monitor");

  /// Log into a subfolder of the data directory (if requested to log)
  auto data_dir_str = node->declare_parameter<std::string>("data_dir", "/tmp");
  fs::path data_dir{utils::expand_user(utils::expand_env(data_dir_str))};

  auto log_to_file = node->declare_parameter<bool>("log_to_file", false);
  auto log_debug = node->declare_parameter<bool>("log_debug", false);
  std::string log_filename;
  if (log_to_file) {
    auto log_name =
        "vtr-safety-monitor-" + timing::toIsoFilename(timing::clock::now());
    log_filename = data_dir / (log_name + ".log");
  }
  configureLogging(log_filename, log_debug, {"safety_monitor"});

  CLOG(INFO, "safety_monitor") << "Safety monitor booting up.";
  SafetyMonitor safety_monitor{node};

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}
