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
 * \file time_delay_compensation.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <deque>

#include <rclcpp/rclcpp.hpp>

#include <vtr_logging/logging.hpp>

namespace vtr {
namespace path_tracker {

/** \brief  */
class MpcTimeDelayComp {
 private:
  /// Struct to hold entries
  typedef struct {
    rclcpp::Time ctrl_time;
    double v_cmd;
    double w_cmd;
  } cmd_hist_entry;

  /// Command History
  std::deque<cmd_hist_entry> cmd_hist;

 public:
  MpcTimeDelayComp() { cmd_hist.clear(); }
  ~MpcTimeDelayComp() = default;

  // Functions
  void clear_hist() { cmd_hist.clear(); }
  bool add_hist_entry(const float &v_cmd, const float &w_cmd,
                      const rclcpp::Time &ctrl_time, rclcpp::Clock &clock);
  bool get_cmd_list(const rclcpp::Time &t_1, const rclcpp::Time &t_2,
                    std::vector<float> &v_cmd_vec,
                    std::vector<float> &w_cmd_vec,
                    std::vector<float> &dt_time_vec, rclcpp::Clock &clock);
  bool get_avg_cmd(const rclcpp::Time &t_1, const rclcpp::Time &t_2,
                   float &v_cmd_avg, float &w_cmd_avg, rclcpp::Clock &clock);
  bool del_hist_older_than(const rclcpp::Time &t_1);

  // Utilities
  int get_size() { return cmd_hist.size(); }
};

}  // namespace path_tracker
}  // namespace vtr
