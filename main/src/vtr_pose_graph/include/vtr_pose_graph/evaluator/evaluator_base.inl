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
 * \file evaluator_base.inl
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/evaluator/evaluator_base.hpp>

namespace vtr {
namespace pose_graph {
namespace eval {

template <class RVAL, class GRAPH>
RVAL CachingBase<RVAL, GRAPH>::operator[](const SimpleVertex &v) {
  auto rval = this->vertexMap_->insert({v, RVAL()});

  if (rval.second)
    rval.first->second = this->computeVertex(this->graph_->at(v));
  return rval.first->second;
}

template <class RVAL, class GRAPH>
RVAL CachingBase<RVAL, GRAPH>::operator[](const SimpleEdge &e) {
  auto rval = this->edgeMap_->insert({e, RVAL()});

  if (rval.second) rval.first->second = this->computeEdge(this->graph_->at(e));
  return rval.first->second;
}

template <class RVAL, class GRAPH>
RVAL CachingBase<RVAL, GRAPH>::operator[](const VertexPtr &v) {
  auto rval = this->vertexMap_->insert({v->simpleId(), RVAL()});

  if (rval.second) rval.first->second = this->computeVertex(v);
  return rval.first->second;
}

template <class RVAL, class GRAPH>
RVAL CachingBase<RVAL, GRAPH>::operator[](const EdgePtr &e) {
  auto rval = this->edgeMap_->insert({e->simpleId(), RVAL()});

  if (rval.second) rval.first->second = this->computeEdge(e);
  return rval.first->second;
}

template <class RVAL, class GRAPH>
RVAL WindowedBase<RVAL, GRAPH>::operator[](const SimpleVertex &v) {
  auto rval = this->vertexMap_->insert({v, RVAL()});

  if (rval.second) {
    rval.first->second = this->computeVertex(this->graph_->at(v));

    vertexQueue_.push_back(v);
    (void)this->vertexMap_->erase(vertexQueue_.front());
    vertexQueue_.pop_front();
  }

  return rval.first->second;
}

template <class RVAL, class GRAPH>
RVAL WindowedBase<RVAL, GRAPH>::operator[](const SimpleEdge &e) {
  auto rval = this->edgeMap_->insert({e, RVAL()});

  if (rval.second) {
    rval.first->second = this->computeEdge(this->graph_->at(e));

    edgeQueue_.push_back(e);
    (void)this->edgeMap_->erase(edgeQueue_.front());
    edgeQueue_.pop_front();
  }

  return rval.first->second;
}

template <class RVAL, class GRAPH>
RVAL WindowedBase<RVAL, GRAPH>::operator[](const VertexPtr &v) {
  auto rval = this->vertexMap_->insert({v->simpleId(), RVAL()});

  if (rval.second) {
    rval.first->second = this->computeVertex(v);

    vertexQueue_.push_back(v->simpleId());
    (void)this->vertexMap_->erase(vertexQueue_.front());
    vertexQueue_.pop_front();
  }

  return rval.first->second;
}

template <class RVAL, class GRAPH>
RVAL WindowedBase<RVAL, GRAPH>::operator[](const EdgePtr &e) {
  auto rval = this->edgeMap_->insert({e->simpleId(), RVAL()});

  if (rval.second) {
    rval.first->second = this->computeEdge(e);

    edgeQueue_.push_back(e->simpleId());
    (void)this->edgeMap_->erase(edgeQueue_.front());
    edgeQueue_.pop_front();
  }

  return rval.first->second;
}

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr
