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
 * \file time_delta_planner.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_path_planning/planning_interface.hpp>
#if 0
#include <vtr_pose_graph/evaluator/time.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#endif

namespace vtr {
namespace path_planning {
#if 0
using vtr::pose_graph::RCGraph;
using vtr::pose_graph::RCGraphBase;

/// \brief Planner that assignes cost based on time-of-day proximity to
/// experiences
class TimeDeltaPlanner : public PlanningInterface {
 public:
  struct Config {
    double timeFalloff_;
    double daysFalloff_;
    double maxContrib_;
    double smoothing_;
    int maxRuns_;
    bool invert_;
    std::chrono::hours utcOffset_;
    std::chrono::minutes max_cache_age_;
  };

  typedef vtr::pose_graph::Eval::Weight::TimeDelta<RCGraphBase>::Caching
      EvalType;

  using time_point = vtr::common::timing::time_point;
  using clock = vtr::common::timing::clock;

  PTR_TYPEDEFS(TimeDeltaPlanner)

  TimeDeltaPlanner(const RCGraphBase::Ptr& graph,
                   const Config& config = Config(),
                   const time_point& start = clock::now());

  /// \brief Get a path between two vertices
  virtual PathType path(const VertexId& from, const VertexId& to);

  /// \brief Get a path to a set of vertices
  virtual PathType path(const VertexId& from, const VertexId::List& to,
                        std::list<uint64_t>* idx = nullptr);

  /// \brief Update the cached privileged graph
  virtual void updatePrivileged();

  /// \brief Get the cost of a vertex
  virtual double cost(const VertexId& v) { return eval_->operator[](v); }

  /// \brief Get the cost of an edge
  virtual double cost(const EdgeId& e) { return eval_->operator[](e); }

  /// \brief Get the cost evaluator itself
  virtual EvalPtr cost() {
    std::lock_guard<std::mutex> lck(cost_mtx_);
    _resetEval();
    return eval_;
  }

 private:
  /// \brief Internal, single segment path planner.  Called by both public
  /// functions.
  PathType _path(const VertexId& from, const VertexId& to);

  /// \brief Purges cached costs if too much time has passed
  inline void _checkCache() {
    auto now = clock::now();
    if (cached_from_ - now > config_.max_cache_age_) {
      this->_resetEval(now);
    }
  }

  /// \brief Purges cached costs by replacing the internal evaluator
  inline void _resetEval(const time_point& time = clock::now()) {
    vtr::pose_graph::Eval::Weight::TimeDeltaConfig conf(
        time, config_.utcOffset_, config_.timeFalloff_, config_.daysFalloff_,
        config_.maxContrib_, config_.smoothing_, config_.invert_,
        config_.maxRuns_);
    eval_.reset(new EvalType(conf));
    eval_->setGraph(graph_.get());
    cached_from_ = time;
  }

  typename RCGraphBase::Ptr graph_;

  typename RCGraphBase::Ptr privileged_;

  typename EvalType::Ptr eval_;

  time_point cached_from_;

  const Config config_;

  std::mutex cost_mtx_;
};
#endif
}  // namespace path_planning
}  // namespace vtr
