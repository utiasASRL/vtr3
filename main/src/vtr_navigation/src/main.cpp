// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file main.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "vtr_common/timing/utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_navigation/navigator.hpp"

namespace fs = std::filesystem;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::navigation;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("navigator");

  // // Read setup file
  // node->declare_parameter<std::string>("data_dir", "/tmp");
  // if (node->get_parameter("data_dir").get_value<std::string>() == ""){
  //   std::string data_dir_str;
  //   try {
  //     YAML::Node setup_params = YAML::LoadFile(std::getenv("VTRTEMP") + std::string("/setup_params.yaml"));
  //     data_dir_str = setup_params["data_dir"].as<std::string>();
  //   }
  //   catch (const YAML::Exception& e) {
  //     CLOG(INFO, "navigation") << "Reading YAML setup file failed, using default params: " << e.what();
  //     data_dir_str = "/tmp";
  //   }

  //   rclcpp::ParameterValue data_dir_value(data_dir_str);
  //   node->set_parameter(rclcpp::Parameter("data_dir", data_dir_value));
  // }

  // /// Setup logging
  // auto data_dir_str = node->get_parameter("data_dir").get_value<std::string>();
  auto data_dir_str = node->declare_parameter<std::string>("data_dir", "/tmp");
  fs::path data_dir{utils::expand_user(utils::expand_env(data_dir_str))};

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

  // disable eigen multi-threading
  Eigen::setNbThreads(1);

  /// Navigator node that launches and runs the whole vtr navigation system
  Navigator navigator{node};

  /// Run the node
  // 3 threads: 1 for sensor input, 1 for mission planning server (user commands
  // for teach, repeat, etc), 1 for graph map server (graph&tactic callbacks,
  // graph manipulation commands, etc)
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    /* num_threads */ 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}