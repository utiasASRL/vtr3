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
 * \file pose_graph_optimizer.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "steam.hpp"

#include "vtr_pose_graph/optimization/privileged_frame.hpp"

namespace vtr {
namespace pose_graph {

template <class Graph>
class PGOFactorInterface {
 public:
  PTR_TYPEDEFS(PGOFactorInterface);

  using GraphPtr = typename Graph::Ptr;
  using VertexId2TransformMap = std::unordered_map<VertexId, EdgeTransform>;
  using StateMap = std::unordered_map<VertexId, steam::se3::SE3StateVar::Ptr>;
  using CostTerms = std::vector<steam::BaseCostTerm::ConstPtr>;

  virtual ~PGOFactorInterface() = default;

  /** \brief Adds state variables and cost terms to the optimization problem */
  virtual void addCostTerms(const GraphPtr&, StateMap&, CostTerms&) = 0;
};

template <class Graph>
class PoseGraphOptimizer {
 public:
  PTR_TYPEDEFS(PoseGraphOptimizer);

  using GraphPtr = typename Graph::Ptr;
  using VertexPtr = typename Graph::VertexPtr;
  using EdgePtr = typename Graph::EdgePtr;
  using VertexId2TransformMap = std::unordered_map<VertexId, EdgeTransform>;
  using StateMap = std::unordered_map<VertexId, steam::se3::SE3StateVar::Ptr>;
  using CostTerms = std::vector<steam::BaseCostTerm::ConstPtr>;

  /**
   * \brief automatically initializes vertices to tree expansion
   * \note this class is not thread safe, do not use it on a changing graph
   */
  PoseGraphOptimizer(const GraphPtr& graph, const VertexId& root,
                     VertexId2TransformMap& vid2tf_map);

  /** \brief adds factors to the optimization problem */
  void addFactor(const typename PGOFactorInterface<Graph>::Ptr& factor);

  /** \brief Solve the optimization problem using a given solver */
  template <class Solver>
  void optimize(
      const typename Solver::Params& params = typename Solver::Params());

  /** \brief Get a transform by vertex ID */
  const lgmath::se3::Transformation& at(const VertexId& v) const {
    return state_map_.at(v)->value();
  };

 private:
  /** \brief This class modifies this map after optimizing */
  VertexId2TransformMap& vid2tf_map_;

 private:
  GraphPtr graph_;
  StateMap state_map_;
  CostTerms cost_terms_;
};

template <class Graph>
PoseGraphOptimizer<Graph>::PoseGraphOptimizer(const GraphPtr& graph,
                                              const VertexId& root,
                                              VertexId2TransformMap& vid2tf_map)
    : graph_(graph), vid2tf_map_(vid2tf_map) {
  // fill in any missing entries in the tf map
  updatePrivilegedFrame<Graph>(graph_, root, vid2tf_map);

  // initialize states and lock the root
  for (auto iter = graph_->begin(root); iter != graph_->end(); ++iter) {
    const auto vid = iter->v()->id();
    state_map_[vid] = steam::se3::SE3StateVar::MakeShared(vid2tf_map_.at(vid));
  }
  state_map_.at(root)->locked() = true;
}

template <class Graph>
void PoseGraphOptimizer<Graph>::addFactor(
    const typename PGOFactorInterface<Graph>::Ptr& factor) {
  factor->addCostTerms(graph_, state_map_, cost_terms_);
}

template <class Graph>
template <class Solver>
void PoseGraphOptimizer<Graph>::optimize(
    const typename Solver::Params& params) {
  steam::OptimizationProblem problem;

  int num_state_vars = 0;
  for (auto&& it : state_map_) {
    if (!it.second->locked()) {
      problem.addStateVariable(it.second);
      ++num_state_vars;
    } else
      CLOG(INFO, "pose_graph") << "PGO skipping locked pose " << it.first;
  }

  for (const auto& cost_term : cost_terms_) problem.addCostTerm(cost_term);

  if (num_state_vars == 0 || cost_terms_.size() == 0) {
    CLOG(INFO, "pose_graph") << "Attempted relaxation on an empty problem...";
    return;
  } else if (problem.cost() < 1) {
    CLOG(INFO, "pose_graph") << "Skipping relaxation because cost too low (<1)";
    return;
  }

  Solver solver(problem, params);
  solver.optimize();

  // update the tf map
  for (auto iter = vid2tf_map_.begin(); iter != vid2tf_map_.end(); ++iter)
    iter->second = state_map_.at(iter->first)->value();
}

}  // namespace pose_graph
}  // namespace vtr