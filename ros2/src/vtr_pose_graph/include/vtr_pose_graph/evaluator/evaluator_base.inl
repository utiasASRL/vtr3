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
