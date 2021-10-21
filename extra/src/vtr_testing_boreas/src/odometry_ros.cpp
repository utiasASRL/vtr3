#include <filesystem>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"

#include "vtr_common/timing/time_utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_navigation/navigator.hpp"

using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::navigation;
using namespace vtr::tactic;

namespace {
EdgeTransform loadTransform(std::string source_frame,
                            std::string target_frame) {
  rclcpp::Clock::SharedPtr clock =
      std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer tf_buffer{clock};
  tf2_ros::TransformListener tf_listener{tf_buffer};
  if (tf_buffer.canTransform(source_frame, target_frame, tf2::TimePoint(),
                             tf2::durationFromSec(1))) {
    auto tf_source_target = tf_buffer.lookupTransform(
        source_frame, target_frame, tf2::TimePoint(), tf2::durationFromSec(1));
    tf2::Stamped<tf2::Transform> tf2_source_target;
    tf2::fromMsg(tf_source_target, tf2_source_target);
    lgmath::se3::TransformationWithCovariance T_source_target(
        common::rosutils::fromStampedTransformation(tf2_source_target));
    T_source_target.setCovariance(Eigen::Matrix<double, 6, 6>::Zero());
    CLOG(DEBUG, "navigator")
        << "Transform from " << target_frame << " to " << source_frame
        << " has been set to" << T_source_target;
    return T_source_target;
  }
  CLOG(WARNING, "navigator")
      << "Transform not found - source: " << source_frame
      << " target: " << target_frame << ". Default to identity.";
  EdgeTransform T_source_target(Eigen::Matrix4d(Eigen::Matrix4d::Identity()));
  T_source_target.setCovariance(Eigen::Matrix<double, 6, 6>::Zero());
  return T_source_target;
}
}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("navigator");

  // Output directory
  const auto data_dir_str =
      node->declare_parameter<std::string>("data_dir", "/tmp");
  fs::path data_dir{utils::expand_user(utils::expand_env(data_dir_str))};

  // Configure logging
  const auto log_to_file = node->declare_parameter<bool>("log_to_file", false);
  const auto log_debug = node->declare_parameter<bool>("log_debug", false);
  const auto log_enabled = node->declare_parameter<std::vector<std::string>>(
      "log_enabled", std::vector<std::string>{});
  std::string log_filename;
  if (log_to_file) {
    // Log into a subfolder of the data directory (if requested to log)
    auto log_name = "vtr-" + timing::toIsoFilename(timing::clock::now());
    log_filename = data_dir / (log_name + ".log");
  }
  configureLogging(log_filename, log_debug, log_enabled);

  // Pose graph
  auto graph = tactic::Graph::MakeShared((data_dir / "graph").string(), false);

  // Pipeline
  auto pipeline_factory = std::make_shared<ROSPipelineFactory>(node);
  pipeline_factory->add<lidar::LidarPipeline>();
  auto pipeline = pipeline_factory->make("pipeline");

  // Tactic
  auto tactic = std::make_shared<Tactic>(Tactic::Config::fromROS(node), node,
                                         pipeline, graph);

  tactic->setPipeline(PipelineMode::Branching);
  tactic->addRun();

  // Frame and transforms
  std::string robot_frame =
      node->declare_parameter<std::string>("robot_frame", "robot");
  std::string lidar_frame =
      node->declare_parameter<std::string>("lidar_frame", "velodyne");

  const auto T_lidar_robot = loadTransform(lidar_frame, robot_frame);
  // lidar pointcloud data subscription
  const auto lidar_topic =
      node->declare_parameter<std::string>("lidar_topic", "/points");

  const auto result_pub =
      node->create_publisher<std_msgs::msg::Bool>("result", 1);

  const auto lidar_sub =
      node->create_subscription<sensor_msgs::msg::PointCloud2>(
          lidar_topic, rclcpp::SystemDefaultsQoS(),
          [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            CLOG(DEBUG, "navigator") << "Received a lidar pointcloud.";

            // Convert message to query_data format and store into query_data
            auto query_data = std::make_shared<lidar::LidarQueryCache>();

            /// \todo (yuchen) need to distinguish this with stamp
            query_data->rcl_stamp.emplace(msg->header.stamp);

            // set time stamp
            Timestamp stamp =
                msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
            query_data->stamp.emplace(stamp);

            // put in the pointcloud msg pointer into query data
            query_data->pointcloud_msg = msg;

            // fill in the vehicle to sensor transform and frame names
            query_data->robot_frame.emplace(robot_frame);
            query_data->lidar_frame.emplace(lidar_frame);
            query_data->T_s_r.emplace(T_lidar_robot);

            //
            if (tactic)
              tactic->runPipeline(query_data);
            else {
              LOG(ERROR) << "Tactic has already been reset.";
            }

            result_pub->publish(std_msgs::msg::Bool());
          }

      );

  // ros2 topic pub --once /terminate std_msgs/msg/Bool "{data: true}"
  const auto terminate_sub = node->create_subscription<std_msgs::msg::Bool>(
      "/terminate", rclcpp::SystemDefaultsQoS(),
      [&](const std_msgs::msg::Bool::SharedPtr) {
        LOG(WARNING) << "Saving pose graph and reset.";
        graph->save();
        tactic.reset();
        graph.reset();
        LOG(WARNING) << "Saving pose graph and reset. - done!";
      });

  rclcpp::spin(node);

  rclcpp::shutdown();
}