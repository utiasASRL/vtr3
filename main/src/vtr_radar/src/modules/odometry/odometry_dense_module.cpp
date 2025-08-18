// This module essentially just do radar_gp_state_estimation.cpp but populate the qdata in the end for the correct values
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "yaml-cpp/yaml.h"
// #include "vtr_radar/modules/odometry/dense_utils/gp_doppler.hpp"
#include "vtr_radar/modules/odometry/odometry_dense_module.hpp"


namespace vtr {
namespace radar {

using namespace tactic;

auto OdometryDenseModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                        const std::string &param_prefix)
    -> ConstPtr {

    auto config = std::make_shared<Config>();
    // clang-format off
    CLOG(DEBUG, "radar.odometry_dense") << "Loading dense odometry parameters";
    // I want to know what param_prefix is
    CLOG(DEBUG, "radar.odometry_dense") << "Param prefix is: " << param_prefix;
    config->doppler_cost = node->declare_parameter<bool>(param_prefix + ".estimation" + ".doppler_cost", config->doppler_cost);
    // lets log this
    CLOG(DEBUG, "radar.odometry_dense") << "The config param: doppler_cost is: " << config->doppler_cost;
    


    return config;

    }

void OdometryDenseModule::run_(QueryCache &qdata0, OutputCache &, const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  // Do nothing if qdata does not contain any radar data (was populated by gyro)
  if(!qdata.radar_data)
  {
    CLOG(WARNING, "radar.odometry_dense") << "No radar data available";
    return;
  }

  //  CLOG(DEBUG, "radar.odometry_dense") << "The config param: doppler_cost is: " << config_->doppler_cost;

  return;

  // the brain of the code goes here I need gyro data as well how do I approach this

}

}  // namespace radar
}  // namespace vtr