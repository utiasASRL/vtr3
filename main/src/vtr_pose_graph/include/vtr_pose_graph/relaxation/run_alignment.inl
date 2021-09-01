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
 * \file run_alignment.inl
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/relaxation/run_alignment.hpp>
//#include <vtr_pose_graph/relaxation/privileged_frame.hpp>
//#include <vtr_pose_graph/evaluator/accumulators.hpp>

namespace vtr {
namespace pose_graph {
#if 0
/////////////////////////////////////////////////////////////////////////////
/// @brief Constructor; automatically initializes vertices to tree expansion
/////////////////////////////////////////////////////////////////////////////
template <class G>
RunAlignment<G>::RunAlignment(const GraphPtr& graph) : graph_(graph) {
  for (auto&& run : graph_->runs()) {
    if (!run.second->isManual()) {
      _buildRun(run.second);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Internal function to expand a single run
/////////////////////////////////////////////////////////////////////////////
template <class G>
void RunAlignment<G>::_buildRun(const RunPtr& run) {
  if (run->vertices().size() == 0 || run->edges(Spatial).size() == 0) {
    LOG(WARNING) << "Skipping run " << run->id()
                 << " due to lack of vertices/edges";
    return;
  }

  using namespace steam::se3;
  StateMapType states;

  VertexId root = VertexId::Invalid();
  // Start with any cached transforms, and make one of those vertices the root
  //  for (auto &&it: run->vertices()) {
  //    if (it.second->hasTransform()) {
  //      states.emplace(it.first, TransformStateVar::Ptr(new
  //      TransformStateVar(it.second->T()))); if (!root.isSet()) root =
  //      it.first;
  //    }
  //  }

  // If no transforms were previously set, find the first vertex with a spatial
  // edge to be the root
  if (!root.isSet()) {
    for (auto&& it : run->vertices()) {
      if (it.second->spatialEdges().size() > 0) {
        root = it.first;
        break;
      }
    }

    // A disconnected run should never happen, so we need to be loud about this
    if (!root.isSet()) {
      LOG(FATAL) << "Run " << run->id()
                 << " has no connection to the rest of the graph!!! (you done "
                    "broke the graph)";
      return;
    }

    // Get the spatial edge and the corresponding privileged vertex
    auto edge = graph_->at(*graph_->at(root)->spatialEdges().begin());
    auto v = graph_->at(*graph_->at(root)->spatialNeighbours().begin());

    // If it doens't have a transform, something is really busted :/
    TransformType T_v(true);
    if (v->hasTransform()) {
      T_v = v->T();
    } else {
      LOG(FATAL) << "A privileged vertex didn't have a cached transform... Did "
                    "this graph come from VT&R 2.0?";
    }

    // Account for edge direction swapping
    TransformType T_init(true);
    if (edge->from() == root) {
      T_init = edge->T().inverse() * T_v;
    } else {
      T_init = edge->T() * T_v;
    }

    // Ensure the root vertex has a state
    states.emplace(root, TransformStateVar::Ptr(new TransformStateVar(T_init)));
  }

  // If we didn't have transforms for all of the states, generate the rest by
  // expansion
  if (states.size() < run->vertices().size()) {
    // Forward pass from root to end
    auto it = ++run->vertices().find(root);
    VertexId last = root;

    for (; it != run->vertices().end(); ++it) {
      VertexId vid = it->first;
      if (states.find(vid) != states.end()) {
        last = vid;
        continue;
      }

      auto e = run->at(EdgeIdType(last, vid, Temporal));
      states.emplace(vid, TransformStateVar::Ptr(new TransformStateVar(
                              e->T() * states.at(last)->getValue())));
      last = vid;
    }

    // Reverse pass from root to beginning
    auto jt = std::reverse_iterator<decltype(it)>(run->vertices().find(root));
    last = root;

    for (; jt != run->vertices().rend(); ++jt) {
      VertexId vid = jt->first;
      if (states.find(vid) != states.end()) {
        last = vid;
        continue;
      }

      auto e = run->at(EdgeIdType(vid, last, Temporal));
      states.emplace(vid, TransformStateVar::Ptr(new TransformStateVar(
                              e->T().inverse() * states.at(last)->getValue())));
      last = vid;
    }
  }

  // If the run isn't a linear chain, something is very wrong
  if (states.size() < run->vertices().size()) {
    LOG(FATAL) << "Run was not temporally connected!!";
  }

  // Populate the global state map with the states from this run
  stateMap_.emplace(run->id(), states);
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Lock a state variable
/////////////////////////////////////////////////////////////////////////////
template <class G>
void RunAlignment<G>::setLock(const VertexIdType& v, bool locked) {
  stateMap_.at(v.majorId()).at(v)->setLock(locked);
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Lock a vector of state variables
/////////////////////////////////////////////////////////////////////////////
template <class G>
void RunAlignment<G>::setLock(const std::vector<VertexIdType>& v, bool locked) {
  for (auto&& it : v) {
    stateMap_.at(it.majorId()).at(it)->setLock(locked);
  }
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Build the run-to-run cost terms
/////////////////////////////////////////////////////////////////////////////
template <class G>
void RunAlignment<G>::buildCostTerms(const Matrix6d& cov,
                                     const LossFuncPtr& lossFunc) {
  using namespace steam;
  ModelGen model(cov);

  // Loop through all runs separately
  for (auto&& rt : stateMap_) {
    CostTermPtr costs(new ParallelizedCostTermCollection());
    auto prev = rt.second.begin();
    auto first = rt.second.begin()->first;

    // For each vertex (state variable) add cost terms
    for (auto it = rt.second.begin(); it != rt.second.end(); ++it) {
      // One cost term per temporal edge, unless both are locked
      if (it->first != first && !it->second->isLocked() &&
          !prev->second->isLocked()) {
        auto edge = graph_->at(prev->first, it->first);
        TransformErrorEval::Ptr errorFunc(
            new TransformErrorEval(edge->T(), it->second, prev->second));
        WeightedLeastSqCostTerm<6, 6>::Ptr cost(
            new WeightedLeastSqCostTerm<6, 6>(errorFunc, model(edge->T()),
                                              lossFunc));
        costs->add(cost);
      }

      prev = it;

      // Don't add cost terms to locked state variables, as they won't be in the
      // problem
      if (it->second->isLocked()) continue;

      // One cost term for each spatial neighbour
      for (auto&& jt : graph_->at(it->first)->spatialNeighbours()) {
        // ...But only if the neighbour is actually in the graph and has a
        // transform
        if (graph_->contains(jt) && graph_->at(jt)->hasTransform()) {
          auto T_v_w = graph_->at(jt)->T();
          auto edge = graph_->at(it->first, jt);

          // Handle edge inversion if necessary
          if (edge->from() == it->first) {
            T_v_w = edge->T().inverse() * T_v_w;
          } else {
            T_v_w = edge->T() * T_v_w;
          }

          // Add a spatial edge constraint between this run vertex and it's
          // localized neighbour
          se3::TransformEvaluator::Ptr eval(
              new se3::TransformStateEvaluator(it->second));
          TransformErrorEval::Ptr errorFunc(
              new TransformErrorEval(T_v_w, eval));
          WeightedLeastSqCostTerm<6, 6>::Ptr cost(
              new WeightedLeastSqCostTerm<6, 6>(errorFunc, model(T_v_w),
                                                lossFunc));

          costs->add(cost);
        }
      }
    }

    costTerms_.emplace(rt.first, costs);
  }
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Solve the optimization problem using a given solver
/////////////////////////////////////////////////////////////////////////////
template <class G>
template <class Solver>
void RunAlignment<G>::optimize(const typename Solver::Params& params) {
  if (params.maxIterations == 0) {
    LOG(INFO) << "Graph optimization skipped since maxIterations==0.";
    return;
  }

  // Each run is a separate problem because they are disjoint when conditioned
  // on the privileged runs
  for (auto&& rt : stateMap_) {
    // Only add unlocked states or STEAM complains
    steam::OptimizationProblem problem;
    for (auto&& it : rt.second) {
      if (!it.second->isLocked()) problem.addStateVariable(it.second);
    }
    problem.addCostTerm(costTerms_.at(rt.first));

    LOG(DEBUG) << "Optimizing run " << rt.first << " with "
               << problem.getStateVariables().size() << "/" << rt.second.size()
               << " states and " << problem.getNumberOfCostTerms()
               << " terms...";

    // Ensure the problem isn't empty, or STEAM also complains
    if (problem.getStateVariables().size() == 0 ||
        problem.getNumberOfCostTerms() == 0) {
      LOG(INFO) << "Attempted relaxation on an empty/locked run (" << rt.first
                << ")... Result: 0 cost; perfect 5/7!";
      continue;
    }

    Solver s(&problem, params);

    try {
      s.optimize();
    } catch (const steam::unsuccessful_step& e) {
      LOG(INFO) << "Unsuccessful step for run " << rt.first
                << " (maybe the stored initial guess is good?)";
    } catch (const steam::decomp_failure& e) {
      LOG(INFO) << "Dcomp failure for fun " << rt.first
                << " (was the run floating?)";
    }
  }
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Get a transform by vertex ID
/////////////////////////////////////////////////////////////////////////////
template <class G>
const lgmath::se3::Transformation& RunAlignment<G>::at(
    const VertexIdType& v) const {
  return stateMap_.at(v.majorId()).at(v)->getValue();
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Apply the optimization to the graph
/////////////////////////////////////////////////////////////////////////////
template <class G>
void RunAlignment<G>::apply() const {
  for (auto&& rt : stateMap_) {
    for (auto&& it : rt.second) {
      graph_->at(it.first)->setTransform(it.second->getValue());
    }
  }
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Get the transform between two vertex IDs
/////////////////////////////////////////////////////////////////////////////
template <class G>
lgmath::se3::Transformation RunAlignment<G>::T_ab(
    const VertexIdType& v_a, const VertexIdType& v_b) const {
  return stateMap_.at(v_a.majorId()).at(v_a)->getValue() /
         stateMap_.at(v_b.majorId()).at(v_b)->getValue();
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Apply the optimization to the graph
/////////////////////////////////////////////////////////////////////////////
template <class G>
void RunAlignment<G>::applyEdge() const {
  for (auto it = graph_->beginEdge(), ite = graph_->endEdge(); it != ite;
       ++it) {
    if (stateMap_.find(it->from().majorId()) != stateMap_.end() &&
        stateMap_.find(it->to().majorId()) != stateMap_.end()) {
      graph_->at(it->id())->setTransform(this->T_ab(it->from(), it->to()));
    }
  }
}
#endif
}  // namespace pose_graph
}  // namespace vtr
