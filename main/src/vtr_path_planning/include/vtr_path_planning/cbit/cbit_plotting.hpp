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
 * \file cbit_plotting.hpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

// Contains Matplotlibcpp debug plotting tools for the cbit planner
// This library is not meant to be included in the main vt&r branch, debug only

#include <memory>
#include <map>
#include "matplotlibcpp.h" // experimental plotting
#include "vtr_path_planning/cbit/utils.hpp"

#pragma once

void plot_tree(Tree current_tree, Node robot_pq, std::vector<double> path_p, std::vector<double> path_q, std::vector<std::shared_ptr<Node>> samples);
void plot_robot(Node robot_pq);
void initialize_plot();