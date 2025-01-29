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
 * \file topological_localize.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_mission_planning/state_machine/states/repeat.hpp"

namespace vtr {
namespace mission_planning {
namespace repeat {
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
std::vector<std::string> get_pointcloud_files(const std::string& base_directory);
std::vector<std::vector<double>> read_matrix_from_txt(const std::string& file_path, int rows, int cols);
double scan_matching();

class TopologicalLocalize : public Repeat {
 public:
  PTR_TYPEDEFS(TopologicalLocalize);
  INHERITANCE_TESTS(TopologicalLocalize, StateInterface);
  using Parent = Repeat;

  std::string name() const override { return Parent::name() + "::TopoLoc"; }
  PipelineMode pipeline() const override { return PipelineMode::RepeatTopLoc; }
  StateInterface::Ptr entryState() const override;
  StateInterface::Ptr nextStep(const StateInterface &) const override;
  void processGoals(StateMachine &, const Event &) override;
  void onExit(StateMachine &, StateInterface &) override;
  void onEntry(StateMachine &, StateInterface &) override;
};

}  // namespace repeat
}  // namespace mission_planning
}  // namespace vtr
