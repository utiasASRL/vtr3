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
 * \file mission_server_node.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include "rclcpp/rclcpp.hpp"

#include <vtr_logging/logging_init.hpp>
#include <vtr_mission_planning/ros_mission_server.hpp>
#include <vtr_mission_planning/test_utils.hpp>

using namespace vtr::logging;
using namespace vtr::mission_planning;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mission_server");

  configureLogging();

  StateMachine::Ptr sm{StateMachine::InitialState()};
  TestTactic::Ptr tactic{new TestTactic{}};
  sm->setTactic(tactic.get());
  sm->setPlanner(TestPathPlanner::Ptr{new TestPathPlanner{}});

  RosMissionServer::Ptr server{new RosMissionServer{node, sm}};

  rclcpp::spin(node);
  rclcpp::shutdown();
}