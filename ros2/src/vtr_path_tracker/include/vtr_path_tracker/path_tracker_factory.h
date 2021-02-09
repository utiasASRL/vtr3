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

  typedef std::shared_ptr<Base> (*fptr)(const std::shared_ptr<Graph>, ros::NodeHandle *);
  typedef std::map<std::string, fptr> FactoryMap;
  FactoryMap funcMap;

  std::shared_ptr<Graph> graph_;
  ros::NodeHandle *nh_;

  /**
   * @brief PathTrackerFactory
   * @param graph: shared pointer to the graph
   * @param nh: raw pointer to the nodehandle
   */
  PathTrackerFactory(const std::shared_ptr<Graph> graph, ros::NodeHandle *nh) : graph_(graph) {
    nh_ = nh;
    funcMap[PathTrackerMPC::type_name] = &PathTrackerMPC::Create; /// REGISTER NEW CONTROLLERS HERE
  }

  std::shared_ptr<Base> create(const std::string &type) {
    auto it = funcMap.find(type);
    if (it != funcMap.end()) {
      return it->second(graph_, nh_);
    } else {
      LOG(ERROR) << "Path tracker of type " << type << " not configured in factory. Returning NULL";
      return nullptr;
    }
  }

};

}
}
