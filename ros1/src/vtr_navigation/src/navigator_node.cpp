// ** FOLLOWING LINE SHOULD BE USED ONCE AND ONLY ONCE IN WHOLE APPLICATION **
// ** THE BEST PLACE TO PUT THIS LINE IS IN main.cpp RIGHT AFTER INCLUDING
// easylogging++.h **
#include <asrl/common/logging.hpp>
#include <asrl/common/logging_config.hpp>
INITIALIZE_EASYLOGGINGPP

#include <ros/ros.h>

#include <vtr/navigation/navigator.h>
// #include <vtr/navigation/tactics/basic_tactic.h>
// #include <vtr/navigation/tactics/parallel_tactic.h>

#include <asrl/common/stacktrace_terminate.hpp>
#include <asrl/common/timing/TimeUtils.hpp>

int main(int argc, char **argv) {
  asrl::common::register_stacktrace_terminate();

  Eigen::initParallel();
  ros::init(argc, argv, "ShamNav");
  ros::NodeHandle nh("~");

  // Log into a subfolder of the data directory (if requested to log)
  std::string log_filename;
  bool to_file;
  nh.param<bool>("log_to_file", to_file, false);
  if (to_file) {
    std::string data_path;
    nh.param<std::string>("data_dir", data_path, "");
    std::string log_name =
        asrl::common::timing::toIsoFilename(asrl::common::timing::clock::now());
    log_filename = data_path + "/logs/" + log_name + ".log";
  }
  asrl::common::logging::configureLogging(log_filename);
  LOG_IF(to_file, INFO) << "Logging to: " << log_filename;
  LOG_IF(!to_file, WARNING) << "NOT LOGGING TO A FILE.";

  LOG(INFO) << "Starting Navigation Node, beep bop boop";
  // THE MAGIC. Wizards only, fools.
  vtr::navigation::Navigator nav(nh);

  // Wait for shutdown
  ros::spin();

  return 0;
}
