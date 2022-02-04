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
 * \file evaluator_base.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <deque>
#include <functional>
#include <unordered_map>

#include "vtr_common/utils/macros.hpp"
#include "vtr_pose_graph/id/id.hpp"

namespace vtr {
namespace pose_graph {
namespace eval {

template <class RVAL>
class BaseEval {
 public:
  PTR_TYPEDEFS(BaseEval);

  virtual ~BaseEval() = default;

  virtual RVAL operator[](const EdgeId &e) { return computeEdge(e); }
  virtual RVAL operator[](const VertexId &v) { return computeVertex(v); }

 private:
  virtual RVAL computeEdge(const EdgeId &e) = 0;
  virtual RVAL computeVertex(const VertexId &v) = 0;
};

/** \brief Evaluator for a function on edge/vertex data, with caching */
template <class RVAL>
class BaseCachedEval : public BaseEval<RVAL> {
 public:
  PTR_TYPEDEFS(BaseCachedEval);

  using Base = BaseEval<RVAL>;

  using EdgeMap = std::unordered_map<EdgeId, RVAL>;
  using VertexMap = std::unordered_map<VertexId, RVAL>;

  RVAL operator[](const EdgeId &e) override {
    auto rval = edge_map_.insert({e, RVAL()});
    if (rval.second) rval.first->second = this->computeEdge(e);
    return rval.first->second;
  }

  RVAL operator[](const VertexId &v) override {
    auto rval = vertex_map_.insert({v, RVAL()});
    if (rval.second) rval.first->second = this->computeVertex(v);
    return rval.first->second;
  }

 protected:
  EdgeMap edge_map_;
  VertexMap vertex_map_;
};

/** \brief Evaluator for a function on edge/vertex data, with a fixed cache */
template <class RVAL>
class BaseWindowedEval : public BaseCachedEval<RVAL> {
 public:
  PTR_TYPEDEFS(BaseWindowedEval);

  using Base = BaseCachedEval<RVAL>;

  using EdgeMap = typename Base::EdgeMap;
  using VertexMap = typename Base::VertexMap;

  BaseWindowedEval(const size_t &size = 500)
      : edge_queue_(size, EdgeId::Invalid()),
        vertex_queue_(size, VertexId::Invalid()) {}

  RVAL operator[](const VertexId &v) override {
    auto rval = this->vertex_map_.insert({v, RVAL()});
    if (rval.second) {
      rval.first->second = this->computeVertex(v);

      vertex_queue_.push_back(v);
      this->vertex_map_.erase(vertex_queue_.front());
      vertex_queue_.pop_front();
    }

    return rval.first->second;
  }

  RVAL operator[](const EdgeId &e) override {
    auto rval = this->edge_map_.insert({e, RVAL()});
    if (rval.second) {
      rval.first->second = this->computeEdge(e);

      edge_queue_.push_back(e);
      this->edge_map_.erase(edge_queue_.front());
      edge_queue_.pop_front();
    }

    return rval.first->second;
  }

 protected:
  std::deque<EdgeId> edge_queue_;
  std::deque<VertexId> vertex_queue_;
};

/** \brief Simple evaluator for a fixed constant weight */
template <class RVAL>
class ConstEval : public BaseEval<RVAL> {
 public:
  PTR_TYPEDEFS(ConstEval);

  using Base = BaseEval<RVAL>;

  ConstEval(const RVAL &edge_value = RVAL(), const RVAL &vertex_value = RVAL())
      : edge_value_(edge_value), vertex_value_(vertex_value) {}

 private:
  RVAL computeEdge(const EdgeId &) override { return edge_value_; }
  RVAL computeVertex(const VertexId &) override { return vertex_value_; }

 private:
  const RVAL edge_value_;
  const RVAL vertex_value_;
};

/** \brief Base evaluator for a fixed map */
template <class RVAL>
class MapEval : public BaseEval<RVAL> {
 public:
  PTR_TYPEDEFS(MapEval);

  using Base = BaseEval<RVAL>;

  using EdgeMap = std::unordered_map<EdgeId, RVAL>;
  using VertexMap = std::unordered_map<VertexId, RVAL>;
  PTR_NAMED_TYPEDEFS(EdgeMap);
  PTR_NAMED_TYPEDEFS(VertexMap);

  MapEval(const EdgeMapPtr &edge_map = std::make_shared<EdgeMap>(),
          const VertexMapPtr &vertex_map = std::make_shared<VertexMap>())
      : edge_map_(edge_map), vertex_map_(vertex_map) {}

  RVAL &ref(const EdgeId &e) { return edge_map_->operator[](e); }
  RVAL &ref(const VertexId &v) { return vertex_map_->operator[](v); }

 private:
  // clang-format off
  RVAL computeEdge(const EdgeId &e) override { return edge_map_->operator[](e); }
  RVAL computeVertex(const VertexId &v) override { return vertex_map_->operator[](v); }
  // clang-format on

 private:
  EdgeMapPtr edge_map_;
  VertexMapPtr vertex_map_;
};

/** \brief Macro to create a new evaluator base type */
#define NEW_EVALUATOR_TYPE(Name, ScalarType)                   \
  namespace Name {                                             \
  using ReturnType = ScalarType;                               \
  using Ptr = std::shared_ptr<eval::BaseEval<ReturnType>>;     \
  using BaseEval = eval::BaseEval<ScalarType>;                 \
  using BaseCachedEval = eval::BaseCachedEval<ScalarType>;     \
  using BaseWindowedEval = eval::BaseWindowedEval<ScalarType>; \
  using ConstEval = eval::ConstEval<ScalarType>;               \
  using MapEval = eval::MapEval<ScalarType>;                   \
  }

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr