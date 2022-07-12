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
        PTR_TYPEDEFS(CBIT);

        static constexpr auto static_name = "cbit_old";

        CBIT(const Config::ConstPtr& config,
                        const RobotState::Ptr& robot_state,
                        const Callback::Ptr& callback);
        ~CBIT() override;

    protected:
        void initializeRoute(RobotState& robot_state) override;
        Command computeCommand(RobotState& robot_state) override;
        //void testFunction() override;

    protected:
        struct ChainInfo {
            tactic::Timestamp stamp;
            Eigen::Matrix<double, 6, 1> w_p_r_in_r;
            // planning frame is the current localization frame
            tactic::EdgeTransform T_p_r;  // T_planning_robot
            tactic::EdgeTransform T_w_p;  // T_world_planning
            tactic::EdgeTransform T_p_g;  // T_planning_goal
            unsigned curr_sid;
        };
        /** \brief Retrieve information for planning from localization chain */
        ChainInfo getChainInfo(RobotState& robot_state);

    private: 
        VTR_REGISTER_PATH_PLANNER_DEC_TYPE(CBIT);

    private:
        const Config::ConstPtr config_;
};
} // namespace path_planning
} // namespace vtr