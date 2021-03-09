#pragma once

#include <vtr_pose_graph/id/id_base.hpp>

namespace vtr {
namespace pose_graph {

DEFINE_PAIRED_ID(EdgeId, Temporal, Spatial)

}  // namespace pose_graph
}  // namespace vtr

// This needs to happen outside the asrl::pose_graph namespace
EXTEND_HASH(vtr::pose_graph::EdgeId)