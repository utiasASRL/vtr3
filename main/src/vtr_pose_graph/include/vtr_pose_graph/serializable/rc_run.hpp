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
 * \file rc_run.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <filesystem>

#include <vtr_pose_graph/index/run_base.hpp>
#include <vtr_pose_graph/serializable/rc_edge.hpp>
#include <vtr_pose_graph/serializable/rc_vertex.hpp>
#include <vtr_pose_graph/utils/hash.hpp>  // hash for std::pair

#include <vtr_pose_graph_msgs/msg/run.hpp>

namespace fs = std::filesystem;

namespace vtr {
namespace pose_graph {

class RCRun : public RunBase<RCVertex, RCEdge> {
 public:
  using RunMsg = vtr_pose_graph_msgs::msg::Run;

  using Name2AccessorMapBase =
      std::unordered_map<std::string,
                         std::shared_ptr<storage::DataStreamAccessorBase>>;
  using Name2AccessorMap = common::SharedLockable<
      std::pair<Name2AccessorMapBase, const std::string>>;
  using Name2AccessorMapPtr = std::shared_ptr<Name2AccessorMap>;

  // Typedefs for the mapping used by the SimpleGraph wrapper
  using VertexPtrMapExtern =
      std::unordered_map<typename VertexType::SimpleIdType, VertexPtr>;
  using EdgePtrMapExtern =
      std::unordered_map<typename EdgeType::SimpleIdType, EdgePtr>;
  // Filter runs when loading
  using RunFilter = std::unordered_set<IdType>;

  // Declare shared pointers for this class
  PTR_TYPEDEFS(RCRun)

  /**
   * \brief Interface to downcast base class pointers
   * \details This allows us to do DerivedPtrType = Type::Cast(BasePtrType)
   */
  PTR_DOWNCAST_OPS(RCRun, RunBase)

  static Ptr MakeShared(const IdType& run_id, const std::string& file_path) {
    return Ptr(new RCRun(run_id, file_path));
  }
  static Ptr MakeShared(const std::string& file_path, const RunMsg& msg,
                        VertexPtrMapExtern& vertex_map,
                        EdgePtrMapExtern& edge_map, const RunFilter& run_filter,
                        const storage::LockableMessage<RunMsg>::Ptr& msg_ptr) {
    return Ptr(
        new RCRun(file_path, msg, vertex_map, edge_map, run_filter, msg_ptr));
  }

  RCRun(const IdType& run_id, const std::string& file_path);
  RCRun(const std::string& file_path, const RunMsg& msg,
        VertexPtrMapExtern& vertex_map, EdgePtrMapExtern& edge_map,
        const RunFilter& run_filter,
        const storage::LockableMessage<RunMsg>::Ptr& msg_ptr);

  // This class involves file IO and cannot/should not be copied.
  // These functions are declared as deleted so that doing so will yield a
  // compile error
  RCRun(const RCRun&) = delete;  // Involves file IO, should not be copied.
  RCRun(RCRun&&) = delete;
  RCRun& operator=(const RCRun&) = delete;
  RCRun& operator=(RCRun&&) = delete;

  /**
   * \brief Destructor
   * \details \note this does not save anything, as move semantics might yield
   *          an empty class that isn't savable.
   */
  virtual ~RCRun() = default;

  /** \brief Write all data to file */
  storage::LockableMessage<RunMsg>::Ptr serialize();

  /** \brief Return a blank vertex with the next available Id */
  VertexPtr addVertex(const Timestamp& time);

  /** \brief Get the path of the top level run file */
  std::string filePath() const { return file_path_; }

  /** \brief Determine if the run is ephemeral, or will be saved */
  bool isEphemeral() const { return file_path_.empty(); }

  /** \brief Writes a message to disk directly, without associated vertex.*/
  template <typename DataType>
  void write(const std::string& stream_name, const std::string& stream_type,
             const typename storage::LockableMessage<DataType>::Ptr& message);

 private:
  /** \brief Load all vertices associated with this run */
  void loadVertices(VertexPtrMapExtern& vertexDataMap,
                    const RunFilter& runFilter);
  /**
   * \brief Load all edges associated with this run
   * \details All edges are loaded, however only edges between filtered runs
   *          are inserted into the return map
   */
  void loadEdges(EdgePtrMapExtern& edgeDataMap, const RunFilter& runFilter);

  /** \brief Save all vertices associated with this run to file */
  void saveVertices();
  /** \brief Save all edges associated with this run to file */
  void saveEdges();

#if false
 protected:
  /** \brief Return a blank vertex with the next available Id */
  virtual const std::shared_ptr<RCVertex>& addVertex(
      const VertexIdType& v = VertexIdType::Invalid());
#endif
 private:
  /** \brief Location of the top-level run file */
  const std::string file_path_;

  Name2AccessorMapPtr name2accessor_map_;

  /** \brief Message structure containing run metadata */
  storage::LockableMessage<RunMsg>::Ptr msg_ = nullptr;

  friend class RCGraph;
  template <typename V, typename E, typename R>
  friend class Graph;
};

template <typename DataType>
void RCRun::write(
    const std::string& stream_name, const std::string& stream_type,
    const typename storage::LockableMessage<DataType>::Ptr& message) {
  // check if exists (with a shared lock so that it does not block other reads)
  {
    const auto name2accessor_map_locked = name2accessor_map_->sharedLocked();
    const auto& name2accessor_map_ref = name2accessor_map_locked.get();

    const auto accessor_itr = name2accessor_map_ref.first.find(stream_name);
    if (accessor_itr != name2accessor_map_ref.first.end()) {
      std::dynamic_pointer_cast<storage::DataStreamAccessor<DataType>>(
          accessor_itr->second)
          ->write(message);
      return;
    }
  }

  // perform insertion to the map
  const auto name2accessor_map_locked = name2accessor_map_->locked();
  auto& name2accessor_map_ref = name2accessor_map_locked.get();
  const auto accessor_itr = name2accessor_map_ref.first.try_emplace(
      stream_name, std::make_shared<storage::DataStreamAccessor<DataType>>(
                       name2accessor_map_ref.second, stream_name, stream_type));
  std::dynamic_pointer_cast<storage::DataStreamAccessor<DataType>>(
      accessor_itr.first->second)
      ->write(message);
}

}  // namespace pose_graph
}  // namespace vtr
