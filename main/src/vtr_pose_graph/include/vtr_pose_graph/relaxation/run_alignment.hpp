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
 * \file run_alignment.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once
#if 0
#include <boost/unordered_map.hpp>
#include <lgmath/se3/TransformationWithCovariance.hpp>
#include <steam.hpp>

#include <vtr_common/logging.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#include <vtr_pose_graph/relaxation/graph_optimization_problem.hpp>
#endif
namespace vtr {
namespace pose_graph {
#if 0
template <class G>
class RunAlignment {
 public:
  typedef typename G::Ptr GraphPtr;

  typedef typename G::VertexPtr VertexPtr;
  typedef typename G::EdgePtr EdgePtr;
  typedef typename G::RunPtr RunPtr;
  typedef typename G::VertexIdType VertexIdType;
  typedef typename G::EdgeIdType EdgeIdType;
  typedef typename G::RunIdType RunIdType;
  typedef typename G::EdgeType::TransformType TransformType;

  typedef std::map<VertexId, steam::se3::TransformStateVar::Ptr> StateMapType;
  typedef std::map<RunIdType, StateMapType> RunStateMap;

  typedef __shared_ptr<steam::OptimizationProblem> ProblemPtr;
  typedef steam::ParallelizedCostTermCollection::Ptr CostTermPtr;
  typedef std::map<RunIdType, CostTermPtr> CostTermMap;

  typedef steam::LossFunctionBase::Ptr LossFuncPtr;
  typedef steam::BaseNoiseModel<6>::Ptr NoiseModelPtr;
  typedef steam::StaticNoiseModel<6> NoiseModel;
  typedef NoiseModelGenerator<TransformType, 6> ModelGen;

  typedef Eigen::Matrix<double, 6, 6> Matrix6d;

  PTR_TYPEDEFS(RunAlignment)
  DEFAULT_COPY_MOVE(RunAlignment)

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Constructor; automatically initializes vertices to tree expansion
  /////////////////////////////////////////////////////////////////////////////
  RunAlignment(const GraphPtr& graph);

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Lock a state variable
  /////////////////////////////////////////////////////////////////////////////
  void setLock(const VertexIdType& v, bool locked = true);

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Lock a vector of state variables
  /////////////////////////////////////////////////////////////////////////////
  void setLock(const std::vector<VertexIdType>& v, bool locked = true);

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Build the run-to-run cost terms
  /////////////////////////////////////////////////////////////////////////////
  void buildCostTerms(
      const Matrix6d& cov = Matrix6d::Identity(6, 6),
      const LossFuncPtr& lossFunc = LossFuncPtr(new steam::L2LossFunc()));

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Solve the optimization problem using a given solver
  /////////////////////////////////////////////////////////////////////////////
  template <class Solver>
  void optimize(
      const typename Solver::Params& params = typename Solver::Params());

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Get a transform by vertex ID
  /////////////////////////////////////////////////////////////////////////////
  const lgmath::se3::Transformation& at(const VertexIdType& v) const;

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Get a transform by vertex ID
  /////////////////////////////////////////////////////////////////////////////
  inline typename StateMapType::const_iterator begin(
      const RunIdType& run) const {
    return stateMap_.at(run).begin();
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Get a transform by vertex ID
  /////////////////////////////////////////////////////////////////////////////
  inline typename StateMapType::const_iterator end(const RunIdType& run) const {
    return stateMap_.at(run).end();
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Apply the optimization to the graph
  /////////////////////////////////////////////////////////////////////////////
  void apply() const;

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Get the transform between two vertex IDs
  /////////////////////////////////////////////////////////////////////////////
  lgmath::se3::Transformation T_ab(const VertexIdType& v_a,
                                   const VertexIdType& v_b) const;

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Apply the optimization to the graph
  /////////////////////////////////////////////////////////////////////////////
  void applyEdge() const;

 protected:
  /////////////////////////////////////////////////////////////////////////////
  /// @brief Internal function to expand a single run
  /////////////////////////////////////////////////////////////////////////////
  void _buildRun(const RunPtr& run);

  GraphPtr graph_;

  RunStateMap stateMap_;

  CostTermMap costTerms_;
};
#endif
}  // namespace pose_graph
}  // namespace vtr

#include <vtr_pose_graph/relaxation/run_alignment.inl>

namespace vtr {
namespace pose_graph {
#if 0
#ifndef RUN_ALIGNMENT_NO_EXTERN
extern template class RunAlignment<BasicGraph>;
extern template class RunAlignment<RCGraph>;
#endif
#endif
}  // namespace pose_graph
}  // namespace vtr