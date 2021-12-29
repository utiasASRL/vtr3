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
 * \file mpc_path_planner.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "rclcpp/rclcpp.hpp"

#include "vtr_path_planning/base_path_planner.hpp"

namespace vtr {
namespace path_planning {

class MPCPathPlanner : public BasePathPlanner {
 public:
  PTR_TYPEDEFS(MPCPathPlanner);

  struct Config {
    PTR_TYPEDEFS(Config);

    unsigned int control_period = 0;

    static UniquePtr fromROS(const rclcpp::Node::SharedPtr& node,
                             const std::string& prefix = "path_planning");
  };

  MPCPathPlanner(Config::UniquePtr config,
                 const Callback::Ptr& callback = std::make_shared<Callback>());
  ~MPCPathPlanner() override;

 private:
  /** \brief Subclass override this method to compute a control command */
  Command computeCommand() override;

 private:
  Config::UniquePtr config_;
};

}  // namespace path_planning
}  // namespace vtr
