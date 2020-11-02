#include <vtr_testing/module_offline.hpp>

//#include <chrono>
//#include <filesystem>

//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
//#include <rclcpp/rclcpp.hpp>

//#include <vtr_common/timing/simple_timer.hpp>
//#include <vtr_navigation/factories/ros_tactic_factory.hpp>
//#include <vtr_vision/messages/bridge.hpp>
//#include <vtr_vision/types.hpp>

// #include <babelfish_robochunk_robochunk_sensor_msgs/RigImages.h>
//#include <vtr_messages/msg/rig_images.hpp>
//#include <vtr_messages/msg/time_stamp.hpp>
//
//#include <vtr_storage/data_stream_reader.hpp>
#if false
#include <robochunk_msgs/XB3CalibrationResponse.pb.h>
#include <vtr_navigation/tactics/basic_tactic.h>
#include <vtr_navigation/tactics/parallel_tactic.h>
#endif

class ModuleVO : public ModuleOffline {
 public:
  ModuleVO(const std::shared_ptr<rclcpp::Node> node, fs::path &results_dir)
      : ModuleOffline(node, results_dir) {

    initializePipeline();
  }

 private:
  void initializePipeline() final {
    // normally the state machine would add a run when a goal is started. We
    // spoof that here.
    tactic_->addRun();

    // Create a branch pipeline.
    tactic_->setPipeline(mission_planning::PipelineType::VisualOdometry);

    // get the co-ordinate frame names
    control_frame_ =
        node_->declare_parameter<std::string>("control_frame", "base_link");
    sensor_frame_ =
        node_->declare_parameter<std::string>("sensor_frame", "front_xb3");

    // Extract the Vehicle->Sensor transformation.
    rclcpp::Clock::SharedPtr clock =
        std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_ros::Buffer tf_buffer{clock};
    tf2_ros::TransformListener tf_listener{tf_buffer};
    auto tf_sensor_vehicle =
        tf_buffer.lookupTransform(sensor_frame_, control_frame_,
                                  tf2::TimePoint(), tf2::durationFromSec(5));
    T_sensor_vehicle_ = fromStampedTransformation(tf_sensor_vehicle);
    tactic_->setTSensorVehicle(T_sensor_vehicle_);
  }

};