#include "rclcpp/rclcpp.hpp"

#include <vtr_common/timing/time_utils.hpp>
#include <vtr_common/utils/filesystem.hpp>
#include <vtr_logging/logging_init.hpp>
#include <vtr_navigation/navigator.hpp>

using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::navigation;
using namespace vtr::tactic;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("navigator");

  auto data_dir_str = node->declare_parameter<std::string>("data_dir", "/tmp");
  fs::path data_dir{utils::expand_user(utils::expand_env(data_dir_str))};
  auto clear_data_dir = node->declare_parameter<bool>("clear_data_dir", false);
  if (clear_data_dir) fs::remove_all(data_dir);

  auto log_to_file = node->declare_parameter<bool>("log_to_file", false);
  auto log_debug = node->declare_parameter<bool>("log_debug", false);
  auto log_enabled =
      node->declare_parameter<std::vector<std::string>>("log_enabled", {});
  std::string log_filename;
  if (log_to_file) {
    // Log into a subfolder of the data directory (if requested to log)
    auto log_name = "vtr-" + timing::toIsoFilename(timing::clock::now());
    log_filename = data_dir / (log_name + ".log");
  }
  configureLogging(log_filename, log_debug, log_enabled);

  // Navigator node that runs everything
  Navigator navigator{node};

  /// Odometry specific

  navigator.tactic()->setPipeline(PipelineMode::Branching);
  navigator.tactic()->addRun();

  // Wait for shutdown
  rclcpp::spin(node);
  rclcpp::shutdown();

  /// Save odometry result
  CLOG(INFO, "navigator") << "Saving odometry results.";
  auto odometry_poses = navigator.tactic()->odometryPoses();
  std::ofstream outstream;
  outstream.open(data_dir / "odometry_result.txt");
  outstream << std::setprecision(6) << std::scientific;
  for (auto pose : odometry_poses) {
    const auto& tmp = pose.matrix();
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 4; j++) {
        outstream << tmp(i, j);
        if (i != 2 || j != 3) outstream << " ";
      }
    outstream << "\n";
  }
  outstream.close();
}