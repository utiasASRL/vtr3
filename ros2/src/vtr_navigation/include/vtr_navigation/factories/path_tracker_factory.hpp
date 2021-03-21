#pragma once

#include <iostream>
#include <string>
#include <map>
#include <functional>

#include <vtr_path_tracker/base.h>
#include <vtr_path_tracker/robust_mpc/mpc/mpc_base.h>

namespace vtr {
namespace path_tracker {

/**
 * @brief The ShapeFactory class: Sub in for the controller factory
 */
class PathTrackerFactory {
 public:

  typedef std::shared_ptr<Base> (*fptr)(const std::shared_ptr<Graph>, const std::shared_ptr<rclcpp::Node>);
  typedef std::map<std::string, fptr> FactoryMap;
  FactoryMap funcMap;

  std::shared_ptr<Graph> graph_;
  const std::shared_ptr<rclcpp::Node> node_;

  /**
   * @brief PathTrackerFactory
   * @param graph: shared pointer to the graph
   * @param node: raw pointer to the node
   */
  PathTrackerFactory(const std::shared_ptr<Graph> graph, const std::shared_ptr<rclcpp::Node> node) : graph_(graph), node_(node) {
    funcMap[PathTrackerMPC::type_name] = &PathTrackerMPC::Create; /// REGISTER NEW CONTROLLERS HERE
  }

  std::shared_ptr<Base> create(const std::string &type) {
    auto it = funcMap.find(type);
    if (it != funcMap.end()) {
      return it->second(graph_, node_);
    } else {
      LOG(ERROR) << "Path tracker of type " << type << " not configured in factory. Returning NULL";
      return nullptr;
    }
  }

};

}
}