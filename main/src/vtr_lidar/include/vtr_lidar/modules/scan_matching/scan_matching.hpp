// Copyright 2023, Autonomous Space Robotics Lab (ASRL)
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
 * \file sample_module.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once
#include "/home/hendrik/ASRL/vtr3/src/main/src/vtr_lidar/include/vtr_lidar/cache.hpp"

#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"
#include <vtr_torch/modules/torch_module.hpp>
#include <vector>

namespace vtr {
namespace lidar {
struct DataPoint {
    double timestamp;
    double x, y, z;
    double qx, qy, qz, qw;
};
std::vector<std::vector<double>> load_embeddings(const std::string& filePath);
std::pair<std::vector<DataPoint>, std::vector<DataPoint>> readDataPoints(const std::string& filePath);
double dot_product(const std::vector<double>& v1, const std::vector<double>& v2);
double magnitude(const std::vector<double>& v);
double cosine_dist(const std::vector<double>& v1, const std::vector<double>& v2);
double euclideanDist(const DataPoint& a, const DataPoint& b);
void init_python_interpreter();
void finalize_python_interpreter();
void call_python_function();
int scan_matching(tactic::QueryCache &qdata0);
void reportCriticalError();
void initialisation();



}  // namespace nn
}  // namespace vtr