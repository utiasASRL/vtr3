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
 * \file simple_planner.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_path_planning/planning_interface.hpp>

namespace vtr {
namespace path_planning {

template <class GRAPH>
class SimplePlanner : public PlanningInterface {
 public:
  PTR_TYPEDEFS(SimplePlanner)
  using GraphBasePtr = decltype(std::declval<GRAPH>().getManualSubgraph());

  SimplePlanner(const typename GRAPH::Ptr &graph) : graph_(graph) {
    updatePrivileged();
  }

  virtual PathType path(const VertexId &from, const VertexId &to);
  virtual PathType path(const VertexId &from, const VertexId::List &to,
                        std::list<uint64_t> *idx = nullptr);

  virtual void updatePrivileged();

  virtual double cost(const VertexId &) { return 1; }
  virtual double cost(const EdgeId &) { return 1; }

  virtual EvalPtr cost() {
    return pose_graph::eval::Weight::Const::MakeShared(1, 1);
  }

 private:
  PathType _path(const VertexId &from, const VertexId &to);

  typename GRAPH::Ptr graph_;

  GraphBasePtr privileged_;
};

}  // namespace path_planning
}  // namespace vtr

#include <vtr_path_planning/simple_planner.inl>
