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
 * \file rc_run.cpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <filesystem>

#include <vtr_pose_graph/serializable/rc_run.hpp>

namespace fs = std::filesystem;

namespace vtr {
namespace pose_graph {

/// newly create we assume id is never changed
RCRun::RCRun(const IdType& run_id, const std::string& file_path)
    : RunBase<RCVertex, RCEdge>(run_id), file_path_(file_path) {
  const auto data = std::make_shared<RunMsg>();
  data->id = id_;
  data->vertex_rpath = "vertex_index";
  data->edge_rpaths.push_back("temporal_edge_index");
  data->edge_rpaths.push_back("spatial_edge_index");
  msg_ = std::make_shared<storage::LockableMessage<RunMsg>>(data);
}

/// load from disk
RCRun::RCRun(const std::string& file_path, const RunMsg& data,
             VertexPtrMapExtern& vertex_map, EdgePtrMapExtern& edge_map,
             const RunFilter& run_filter,
             const storage::LockableMessage<RunMsg>::Ptr& msg)
    : RunBase<RCVertex, RCEdge>(data.id), file_path_(file_path), msg_(msg) {
  loadVertices(vertex_map, run_filter);
  loadEdges(edge_map, run_filter);
  computeManual();
}

storage::LockableMessage<RCRun::RunMsg>::Ptr RCRun::serialize() {
  if (isEphemeral()) return nullptr;
  CLOG(DEBUG, "pose_graph") << "Saving run of id " << id();
  std::shared_lock lock(mutex_);
  saveVertices();
  saveEdges();
  return msg_;  /// no need to check if msg has been updated, it wont change
}

auto RCRun::addVertex(const Timestamp& time) -> VertexPtr {
  return RunBase<RCVertex, RCEdge>::addVertex(time, name2accessor_map_);
}

void RCRun::loadVertices(VertexPtrMapExtern& vertices_ext,
                         const RunFilter& run_filter) {
  CLOG(DEBUG, "pose_graph") << "Loading vertices from message.";

  const auto index_msg = msg_->locked().get().getData();
  const auto file = fs::path{file_path_} / index_msg.vertex_rpath;

  storage::DataStreamAccessor<RCVertex::VertexMsg> accessor(file);
  for (int index = 1;; index++) {
    const auto msg = accessor.readAtIndex(index);
    if (!msg) break;

    auto vertex_msg = msg->locked().get().getData();
    CLOG(DEBUG, "pose_graph") << "Loading message with id: " << vertex_msg.id;

    auto new_ptr =
        RCVertex::MakeShared(vertex_msg, id_, name2accessor_map_, msg);
    vertices_.insert(std::make_pair(new_ptr->id(), new_ptr));

    if (run_filter.size() == 0 || RCVertex::MeetsFilter(vertex_msg, run_filter))
      vertices_ext.insert(std::make_pair(new_ptr->simpleId(), new_ptr));
  }
}

void RCRun::loadEdges(EdgePtrMapExtern& edges_ext,
                      const RunFilter& run_filter) {
  const auto index_msg = msg_->locked().get().getData();

  for (size_t i = 0; i < index_msg.edge_rpaths.size(); ++i) {
    CLOG(DEBUG, "pose_graph") << "Loading edges type " << i << " from message.";

    const auto file = fs::path{file_path_} / index_msg.edge_rpaths[i];

    storage::DataStreamAccessor<RCEdge::EdgeMsg> accessor(file);
    for (int index = 1;; index++) {
      const auto msg = accessor.readAtIndex(index);
      if (!msg) break;

      auto edge_msg = msg->locked().get().getData();
      CLOG(DEBUG, "pose_graph")
          << "Loading message with id: from: " << edge_msg.from_id
          << " to: " << edge_msg.to_id;

      const auto type_idx = edge_msg.type.type;

      if (type_idx >= EdgeIdType::NumTypes()) {
        throw std::invalid_argument(
            "These edge objects don't recognize the edge message types you are "
            "trying to load.");
      }

      auto new_ptr = RCEdge::MakeShared(edge_msg, id_, msg);
      edges_[type_idx].insert(std::make_pair(new_ptr->id(), new_ptr));

      if (run_filter.size() == 0 || RCEdge::MeetsFilter(edge_msg, run_filter))
        edges_ext.insert(std::make_pair(new_ptr->simpleId(), new_ptr));
    }
  }
}

void RCRun::saveVertices() {
  const auto index_msg = msg_->locked().get().getData();
  CLOG(DEBUG, "pose_graph") << "Saving vertices to message.";

  const auto file = fs::path{file_path_} / index_msg.vertex_rpath;
  storage::DataStreamAccessor<RCVertex::VertexMsg> writer{file};
  for (auto it = vertices_.begin(); it != vertices_.end(); ++it)
    writer.write(it->second->serialize());
}

void RCRun::saveEdges() {
  const auto index_msg = msg_->locked().get().getData();
  for (unsigned int i = 0; i < edges_.size(); ++i) {
    CLOG(DEBUG, "pose_graph") << "Saving edges of type " << i << " to message.";
    const auto file = fs::path{file_path_} / index_msg.edge_rpaths[i];
    storage::DataStreamAccessor<RCEdge::EdgeMsg> writer{file};
    for (auto it = edges_[i].begin(); it != edges_[i].end(); ++it)
      writer.write(it->second->serialize());
  }
}

#if false
const std::shared_ptr<RCVertex>& RCRun::addVertex(const VertexIdType& v) {
  auto vertex = RunBase<RCVertex, RCEdge>::addVertex(v);
  // Pass the stream-map.
  vertex->setStreamNameMap(vertexStreamNames_);
  // We have to re-look up because we need to return a direct reference :/
  return vertices_.at(vertex->id());
}
#endif

}  // namespace pose_graph
}  // namespace vtr
