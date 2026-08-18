#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_navigation_msgs/msg/goal_handle.hpp"
#include "vtr_navigation_msgs/msg/mission_command.hpp"
#include "vtr_navigation_msgs/msg/robot_state.hpp"
#include "vtr_navigation_msgs/msg/server_state.hpp"
#include "vtr_navigation_msgs/srv/robot_state.hpp"
#include "vtr_route_planning/route_planning.hpp"
#include "vtr_tactic/types.hpp"

using namespace std::chrono_literals;
using namespace vtr::route_planning;
using namespace vtr::pose_graph;

namespace vtr::multi_robot {

class GraphTestNode : public rclcpp::Node {
 public:
  using MissionCommand = vtr_navigation_msgs::msg::MissionCommand;
  using Goal = vtr_navigation_msgs::msg::GoalHandle;
  using GraphPath = pose_graph::Path<tactic::Graph>;
  using RobotState = vtr_navigation_msgs::msg::RobotState;
  using ServerState = vtr_navigation_msgs::msg::ServerState;
  using RobotStateSrv = vtr_navigation_msgs::srv::RobotState;
  using FutureRobotState = rclcpp::Client<RobotStateSrv>::SharedFuture;

  GraphTestNode() : Node("graph_planning_node") {
    logging::configureLogging("", true,
                              declare_parameter<std::vector<std::string>>(
                                  "log_enabled", std::vector<std::string>{}));

    subs_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = subs_cb_group_;

    client_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    const auto data_dir = declare_parameter<std::string>("data_dir");
    graph_ = tactic::Graph::MakeShared(
        data_dir + "/graph", true, std::make_shared<tactic::Graph::Callback>(),
        true);

    for (auto iter = graph_->beginDfs(VertexId(0, 0), 7.5); iter != graph_->end(); iter++) {
      CLOG(WARNING, "test") << "Iter " << static_cast<VertexId>(*iter);
    }
  }

  private:
    tactic::Graph::Ptr graph_;
    BFSPlanner::Ptr route_planner_;
    GraphPath::Ptr active_path_;

    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr subs_cb_group_;
};
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<vtr::multi_robot::GraphTestNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
