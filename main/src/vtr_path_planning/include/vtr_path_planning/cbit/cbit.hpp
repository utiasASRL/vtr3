#include <cstdio>
#include <iostream>
#include <cmath>
#include <vector>
#include <tuple>
#include <algorithm>
#include <random>
#include <chrono> // For benchmarking
#include <memory>


#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "vtr_path_planning/base_path_planner.hpp"

#pragma once

// Note long term, this class should probably be inherited by the base path planner

namespace vtr {
namespace path_planning {

class CBIT : public BasePathPlanner { // syntax for inherited class
    public:
        std::vector<double> path_x;
        std::vector<double> path_y;
        double sample_box_height;
        double sample_box_width;

        CBIT() = default; // temporary default constructor
    
    protected:
        void initializeRouteTest(RobotState& robot_state) override;

};
} // namespace path_planning
} // namespace vtr