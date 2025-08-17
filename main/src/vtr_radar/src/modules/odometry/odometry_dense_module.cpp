// This module essentially just do radar_gp_state_estimation.cpp but populate the qdata in the end 
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "yaml-cpp/yaml.h"
#include "vtr_radar/modules/odometry/dense_utils/gp_doppler.hpp"
#include "vtr_radar/modules/odometry/odometry_dense_module.hpp"


namespace vtr {
namespace radar {

auto OdometryDenseModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                        const std::string &param_prefix)
    -> ConstPtr {

    auto config = std::make_shared<Config>();
    // clang-format off
    // motion compensation
    config->use_vel_meas = node->declare_parameter<bool>(param_prefix + ".use_vel_meas", config->use_vel_meas);



    
    return config;



    }

   void OdometryDenseModule::run_(QueryCache &qdata0, OutputCache &,
                             const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  // Do nothing if qdata does not contain any radar data (was populated by gyro)
  if(!qdata.radar_data)
  {
    return;
  }

  return;

  // the brain of the code goes here I need gyro data as well how do I approach this

}

}  // namespace radar
}  // namespace vtr