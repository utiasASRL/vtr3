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
 * \file common.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/evaluator_base/common.hpp>

#include <vtr_pose_graph/serializable/rc_graph.hpp>
#include <vtr_pose_graph/serializable/rc_graph_base.hpp>

namespace vtr {
namespace pose_graph {
namespace eval {
namespace Mask {

template <class GRAPH>
class PrivilegedDirect : public virtual Typed<GRAPH>::Direct {
 public:
  using Base = typename Typed<GRAPH>::Direct;
  using Base::graph_;

  PTR_TYPEDEFS(PrivilegedDirect);

  static Ptr MakeShared() { return Ptr(new PrivilegedDirect()); }

  PrivilegedDirect() {}
  virtual ~PrivilegedDirect() {}

  PrivilegedDirect(const PrivilegedDirect &) = default;
  PrivilegedDirect &operator=(const PrivilegedDirect &) = default;
  PrivilegedDirect(PrivilegedDirect &&) = default;
  PrivilegedDirect &operator=(PrivilegedDirect &&) = default;

 protected:
  ReturnType computeEdge(const typename Base::EdgePtr &e) override {
    // Return true if the edge is manual, or the edge is between two privileged
    // vertices (computed recursively)
    return e->isManual() || ((*this)[e->from()] && (*this)[e->to()]);
  }
  ReturnType computeVertex(const typename Base::VertexPtr &v) override {
    typedef typename Base::EdgeIdType::Base::Type EidEnumType;
    // Check to see if any of the connected edges are manual edges
    for (uint32_t i = 0; i < Base::EdgeIdType::Base::NumTypes(); ++i) {
      for (const typename Base::VertexIdType &nvid :
           v->neighbours(EidEnumType(i))) {
        try {
          if (graph_->at(v->id(), nvid)->isManual()) return true;
        } catch (const std::out_of_range &e) {
        }
      }
    }
    return false;
  }

  ReturnType computeEdge(const typename Base::EdgePtr &e) const override {
    /// \todo This is going to involve a lot of stupid repetition if called in a
    /// const scope...
    // Return true if the edge is manual, or the edge is between two privileged
    // vertices (computed recursively)
    return e->isManual() || (this->at(e->from()) && this->at(e->to()));
  }
  ReturnType computeVertex(const typename Base::VertexPtr &v) const override {
    typedef typename Base::EdgeIdType::Base::Type EidEnumType;
    // Check to see if any of the connected edges are manual edges
    for (uint32_t i = 0; i < Base::EdgeIdType::Base::NumTypes(); ++i) {
      for (const typename Base::VertexIdType &nvid :
           v->neighbours(EidEnumType(i))) {
        try {
          if (graph_->at(v->id(), nvid)->isManual()) return true;
        } catch (const std::out_of_range &e) {
        }
      }
    }
    return false;
  }
};

template <class GRAPH>
class PrivilegedCaching : public virtual Typed<GRAPH>::Caching,
                          public virtual PrivilegedDirect<GRAPH> {
 public:
  PTR_TYPEDEFS(PrivilegedCaching);

  static Ptr MakeShared() { return Ptr(new PrivilegedCaching()); }

  PrivilegedCaching() {}
  virtual ~PrivilegedCaching() {}

  PrivilegedCaching(const PrivilegedCaching &) = default;
  PrivilegedCaching &operator=(const PrivilegedCaching &) = default;
  PrivilegedCaching(PrivilegedCaching &&other)
      : Typed<GRAPH>::Caching(std::move(other)),
        PrivilegedDirect<GRAPH>(std::move(other)) {}
  PrivilegedCaching &operator=(PrivilegedCaching &&other) {
    Typed<GRAPH>::Caching::operator=(std::move(other));
    PrivilegedDirect<GRAPH>::operator=(std::move(other));
    return *this;
  }
};

template <class GRAPH>
class PrivilegedWindowed : public virtual Typed<GRAPH>::Windowed,
                           public virtual PrivilegedCaching<GRAPH> {
 public:
  PTR_TYPEDEFS(PrivilegedWindowed);

  static Ptr MakeShared(const size_t &cacheSize) {
    return Ptr(new PrivilegedWindowed(cacheSize));
  }

  PrivilegedWindowed(const size_t &cacheSize)
      : Typed<GRAPH>::Windowed(cacheSize) {}
  virtual ~PrivilegedWindowed() {}

  PrivilegedWindowed(const PrivilegedWindowed &) = default;
  PrivilegedWindowed &operator=(const PrivilegedWindowed &) = default;
  PrivilegedWindowed(PrivilegedWindowed &&other)
      : Typed<GRAPH>::Windowed(std::move(other)),
        PrivilegedCaching<GRAPH>(std::move(other)) {}
  PrivilegedWindowed &operator=(PrivilegedWindowed &&other) {
    Typed<GRAPH>::Windowed::operator=(std::move(other));
    PrivilegedCaching<GRAPH>::operator=(std::move(other));
    return *this;
  }
};

template <class GRAPH>
struct Privileged {
  using Direct = PrivilegedDirect<GRAPH>;
  using Caching = PrivilegedCaching<GRAPH>;
  using Windowed = PrivilegedWindowed<GRAPH>;
};

}  // namespace Mask

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr
