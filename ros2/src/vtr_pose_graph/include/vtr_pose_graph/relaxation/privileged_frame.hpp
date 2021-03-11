#pragma once

#include <map>

#include <vtr_common/utils/macros.hpp>
#include <vtr_pose_graph/index/graph_iterator.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>

namespace vtr {
namespace pose_graph {

template <class GRAPH>
class PrivilegedFrame {
 public:
  using VertexPtr = typename GRAPH::VertexPtr;
  using EdgePtr = typename GRAPH::EdgePtr;
  using VertexIdType = typename GRAPH::VertexIdType;
  using TransformType = typename GRAPH::TransformType;
  using IterType = typename GRAPH::OrderedIter;

  PrivilegedFrame(IterType begin, IterType end, bool lazy, bool cache = true);
  PrivilegedFrame(IterType begin, IterType end,
                  TransformType T_root_world = TransformType(true),
                  bool lazy = true, bool cache = true);
  PrivilegedFrame(const PrivilegedFrame&) = default;
  PrivilegedFrame(PrivilegedFrame&&) = default;

  PrivilegedFrame& operator=(const PrivilegedFrame&) = default;
  PrivilegedFrame& operator=(PrivilegedFrame&&) = default;

  /** \brief Get the global transform of a vertex (computed lazily) */
  const TransformType& operator[](const VertexIdType& v);

  /** \brief Get the global transform of a vertex (v must have been computed) */
  const TransformType& at(const VertexIdType& v) const {
    return tfMap_.at(v);
  }

  /** \brief Force the computation of all transforms now */
  void computeAll();

 protected:
  IterType iter_;
  IterType end_;

  std::map<VertexIdType, TransformType> tfMap_;

  bool useCached_;
};

}  // namespace pose_graph
}  // namespace vtr

#include <vtr_pose_graph/relaxation/privileged_frame.inl>

namespace vtr {
namespace pose_graph {
#if 0
#ifndef PRIVILEGED_FRAME_NO_EXTERN
extern template class PrivilegedFrame<BasicGraph>;
extern template class PrivilegedFrame<RCGraph>;
#endif
#endif
}  // namespace pose_graph
}  // namespace vtr
