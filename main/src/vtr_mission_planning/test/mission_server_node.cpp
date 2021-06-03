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