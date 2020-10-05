#pragma once
#if 0
#include <map>

#include <vtr_common/utils/macros.hpp>
#include <vtr_pose_graph/index/graph_iterator.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#endif
namespace vtr {
namespace pose_graph {
#if 0
template <class GRAPH>
class PrivilegedFrame {
 public:
  typedef typename GRAPH::VertexPtr VertexPtr;
  typedef typename GRAPH::EdgePtr EdgePtr;
  typedef typename GRAPH::VertexIdType VertexIdType;
  typedef typename GRAPH::TransformType TransformType;
  typedef typename GRAPH::OrderedIter IterType;

  DEFAULT_COPY_MOVE(PrivilegedFrame)

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Constructor
  /////////////////////////////////////////////////////////////////////////////
  PrivilegedFrame(IterType begin, IterType end, bool lazy, bool cache = true);

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Constructor
  /////////////////////////////////////////////////////////////////////////////
  PrivilegedFrame(IterType begin, IterType end,
                  TransformType T_root_world = TransformType(true),
                  bool lazy = true, bool cache = true);

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Get the global transform of a vertex (computed lazily)
  /////////////////////////////////////////////////////////////////////////////
  const TransformType& operator[](const VertexIdType& v);

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Get the global transform of a vertex (v must have been computed)
  /////////////////////////////////////////////////////////////////////////////
  const TransformType& at(const VertexIdType& v) const;

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Force the computation of all transforms now
  /////////////////////////////////////////////////////////////////////////////
  void computeAll();

 protected:
  IterType iter_;

  IterType end_;

  std::map<VertexIdType, TransformType> tfMap_;

  bool useCached_;
};
#endif
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
