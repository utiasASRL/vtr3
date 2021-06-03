#pragma once

#include <filesystem>

#include <vtr_pose_graph/index/rc_graph/rc_edge.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_vertex.hpp>
#include <vtr_pose_graph/index/rc_graph/types.hpp>
#include <vtr_pose_graph/index/run_base.hpp>
/// #include <asrl/messages/Run.pb.h>
#include <vtr_messages/msg/graph_run.hpp>
#include <vtr_pose_graph/utils/hash.hpp>  // hash for std::pair

#if 0
#include <robochunk/base/ChunkSerializer.hpp>
#include <robochunk/base/DataInputStream.hpp>
#include <robochunk/base/DataOutputStream.hpp>
#include <robochunk/base/DataStream.hpp>

#include <asrl/messages/Utility.pb.h>
#include <asrl/common/utils/lockable.hpp>
#include <asrl/pose_graph/index/Types.hpp>

#include <queue>
#endif

namespace fs = std::filesystem;

namespace vtr {
namespace pose_graph {
#if 0
template <typename V, typename E, typename R>
class Graph;
class RCGraph;
#endif
class RCRun : public RunBase<RCVertex, RCEdge> {
 public:
  // Structures to map between field names and field ids
  using FieldMap = std::map<std::string, BaseIdType>;
  using LockableFieldMap = common::Lockable<FieldMap>;
  using LockableFieldMapPtr = std::shared_ptr<LockableFieldMap>;
  using LockableFieldMapPtrVector = std::vector<LockableFieldMapPtr>;
  using LockableFieldMapPtrArray =
      std::array<LockableFieldMapPtr, EdgeIdType::NumTypes()>;

  // Structures to map between field ids and data streams. (rosbag2)
  using DataStreamMap = std::map<BaseIdType, RosBagIO>;
  using LockableDataStreamMap = common::Lockable<DataStreamMap>;
  using LockableDataStreamMapPtr = std::shared_ptr<LockableDataStreamMap>;

  // Typedefs for the mapping used by the SimpleGraph wrapper
  using VertexPtrMapExtern =
      std::unordered_map<typename VertexType::SimpleIdType, VertexPtr>;
  using EdgePtrMapExtern =
      std::unordered_map<typename EdgeType::SimpleIdType, EdgePtr>;
  using EdgePtrMapArrayExtern =
      std::array<EdgePtrMapExtern, EdgeIdType::NumTypes()>;

  // Filter runs when loading
  using RunFilter = std::unordered_set<IdType>;

  // Declare shared pointers for this class
  PTR_TYPEDEFS(RCRun)

  /**
   * \brief Interface to downcast base class pointers
   * \details This allows us to do DerivedPtrType = Type::Cast(BasePtrType)
   */
  PTR_DOWNCAST_OPS(RCRun, RunBase)

  /** \brief Convenience constructor to create a shared pointer */
  static Ptr MakeShared() {
    return Ptr(new RCRun());
  }
  static Ptr MakeShared(const IdType& runId, const IdType& graphId) {
    return Ptr(new RCRun(runId, graphId));
  }
  static Ptr MakeShared(const std::string& filePath, const IdType& runId,
                        const IdType& graphId) {
    return Ptr(new RCRun(filePath, runId, graphId));
  }
  static Ptr MakeShared(const std::string& filePath) {
    return Ptr(new RCRun(filePath));
  }

  /** \brief Default constructor, for completeness */
  RCRun();
  RCRun(const IdType& runId, const IdType& graphId);
  RCRun(const std::string& filePath, const IdType& runId,
        const IdType& graphId);
  RCRun(const std::string& filePath);
  RCRun(const RCRun&) = delete;  // Involves file IO, should not be copied.
  RCRun(RCRun&&) = default;

  /**
   * \brief Destructor
   * \details \note this does not save anything, as move semantics might yield
   *          an empty class that isn't savable.
   */
  virtual ~RCRun() = default;

  // This class involves file IO and cannot/should not be copied.
  // These functions are declared as deleted so that doing so will yield a
  // compile error
  RCRun& operator=(const RCRun&) = delete;
  RCRun& operator=(RCRun&&) = default;

#if 1
  void closeWriter(uint32_t stream_idx) {
    // Get the serializer, exit if it doesn't exist.
    auto& data_stream = rosbag_streams_->locked().get().at(stream_idx);
    auto writer = data_stream.second;
    if (writer)
      writer->close();
  }
#endif

  /** \brief Sets all open streams to read only mode. */
  void setReadOnly();
#if 0
  /** \brief Sets a specific stream to read only mode. */
  void setReadOnly(const std::string& stream);

  /**
   * \brief Flushes stream to disk without resetting read stream
   * \detail Useful for flushing a processed stream that does not need to be
   *         read again during same execution.
   */
  void finalizeStream(const std::string& stream);
#endif
  /** \brief Load all graph data */
  void load(VertexPtrMapExtern& vertexDataMap, EdgePtrMapExtern& edgeDataMap,
            const RunFilter& runFilter = RunFilter());

  /**
   * \brief Load the run index file
   * \details This does not clear existing edge and vertex data, and cannot be
   *          used to load a different run file.  Returns true if reindex is
   *          needed for metadata
   */
  bool loadIndex();

  /**
   * \brief Load all edges associated with this run
   * \details All edges are loaded, however only edges between filtered runs
   *          are inserted into the return map
   */
  void loadEdges(EdgePtrMapExtern& edgeDataMap, const RunFilter& runFilter);

  /** \brief Load all vertices associated with this run */
  void loadVertices(VertexPtrMapExtern& vertexDataMap);

#if 0
  /** \brief Write modified data to temporary working files as backup */
  void saveWorking();
  /** \brief Save the modified index to a working file */
  void saveWorkingIndex();

  /** \brief Save all modified edges in this run to temporary files */
  void saveWorkingEdges();

  /** \brief Save all modified vertices in this run to temporary files */
  void saveWorkingVertices();
#endif

  /** \brief Write all data to file */
  void save(bool force = false);

  /** \brief Save the index of this run to file */
  void saveIndex(bool force = false);

  /** \brief Save all vertices associated with this run to file */
  void saveVertices(bool force = false);

  /** \brief Save all edges associated with this run to file */
  void saveEdges(bool force = false);

  /** \brief Get the path of the top level run file */
  std::string filePath() const {
    return filePath_;
  }
#if 0
  /** \brief Set the run file to load; cannot be called after loading */
  void setFilePath(const std::string& fpath);

  /** \brief Returns true if structure was loaded from file */
  bool wasLoaded() const { return wasLoaded_; }

  /** \brief Load a stream of data for all vertices */
  void loadVertexStream(const std::string& streamName, uint64_t start = 0,
                        uint64_t end = 0);

  /** \brief Unload a stream of data for all vertices */
  void unloadVertexStream(const std::string& streamName, uint64_t start = 0,
                          uint64_t end = 0);
#endif

  /** \brief Determine if a stream is registered with the run */
  inline bool hasVertexStream(const std::string& stream) const {
    auto locked_vertex_stream_names = vertexStreamNames_->locked();
    return locked_vertex_stream_names.get().find(stream) !=
           locked_vertex_stream_names.get().end();
  }

  /** \brief Determine if the run is ephemeral, or will be saved */
  inline bool isEphemeral() const {
    return filePath_ == "";
  }

  /** \brief Registers a read/write stream with this run. */
  template <typename MessageType>
  void registerVertexStream(const std::string& path, bool points_to_data = true,
                            const RegisterMode& mode = RegisterMode::Create);

#if 0
  /** \brief Ensure correct vertex indices for a data stream */
  void reindexStream(const std::string& stream, const WindowType& wType,
                     bool overwrite = true);

  /** \brief Raw stream read access for data that isn't vertex-indexed */
  inline const StreamPtr& readStream(const std::string& streamName) const {
    try {
      auto stream_idx = vertexStreamNames_->locked().get().at(streamName);
      return robochunkStreams_->locked().get().at(stream_idx).first;
    } catch (...) {
      LOG(ERROR) << "Stream" << streamName << " not registered on this run!\n"
                 << el::base::debug::StackTrace();
      throw;
    }

    // just so it compiles...
    return robochunkStreams_->locked().get().at(0).first;
  }

  /** \brief Raw stream write access for data that isn't vertex-indexed */
  inline const SerializerPtr& writeStream(const std::string& streamName) const {
    try {
      auto stream_idx = vertexStreamNames_->locked().get().at(streamName);
      return robochunkStreams_->locked().get().at(stream_idx).second;
    } catch (...) {
      LOG(ERROR) << "Stream" << streamName << " not registered on this run!\n"
                 << el::base::debug::StackTrace();
      throw;
    }

    // just so it compiles...
    return robochunkStreams_->locked().get().at(0).second;
  }

  /**
   * \brief Insert a message into the queue for the run.
   * WARNING: Messages logged after the last vertex in a run are lost.
   * See Issue 385 on Github.
   */
  template <typename MessageType>
  bool insert(const std::string& stream_name, const MessageType& message,
              const robochunk::std_msgs::TimeStamp& stamp);

  /** \brief Process messages in the stream queues */
  void processMsgQueue(RCVertex::Ptr vertex);
#endif
  /** \brief Get the robot ID */
  inline IdType robotId() const {
    return robotId_;
  }

  void setRobotId(const IdType& robotId) {
    robotId_ = robotId;
    /// msg_.set_robot_id(robotId_);
    msg_.robot_id = robotId_;
  }

  bool readOnly() {
    return readOnly_;
  }

 protected:
  /** \brief Return a blank vertex with the next available Id */
  virtual const std::shared_ptr<RCVertex>& addVertex(
      const VertexIdType& v = VertexIdType::Invalid());

  /** \brief Load the header message of an edge file to inpect edge type */
  template <class G>
  static size_t loadHeaderInternal(const std::string& fpath);

  /** \brief Loads a vertex or edge file into memory, filtering by run */
  template <class M1, class M2>
  size_t loadDataInternal(M1& dataMap, M2& dataMapInternal,
                          LockableFieldMapPtr& streamNames,
                          const std::string& fpath,
                          const RunFilter& runs = RunFilter());

  /** \brief Generates a header messgage from stream mappings */
  template <class G>
  typename G::HeaderMsg populateHeader(const LockableFieldMapPtr& fields,
                                       const G& example);

  /** \brief Fills in the edge type, if the object id has a type enum */
  template <class M>
  static void populateHeaderEnum(M&, const BaseId&);

  /** \brief Fills in the edge type, if the object id has a type enum */
  template <class M>
  static void populateHeaderEnum(M& msg, const typename EdgeIdType::Base& id);

  /** \brief Saves vertex or edge file to file */
  template <class M>
  void saveDataInternal(M& dataMap, LockableFieldMapPtr& streamNames,
                        const std::string& fpath);
#if 0
  /** \brief Get the path of the working file for a vertex or edge */
  template <class G>
  std::string workingFile(const G& obj, const std::string& basePath);

  /** \brief Saves modified vertices/edges to individual temporary files */
  template <class M>
  void saveWorkingInternal(M& dataMap, const LockableFieldMapPtr& streamNames,
                           const std::string& basePath);
#endif
  /** \brief Map from vertex stream names to integer indices */
  LockableFieldMapPtr vertexStreamNames_;

  /** \brief Map from edge stream names to integer indices */
  LockableFieldMapPtrArray edgeStreamNames_;

  /** \brief Map from field ids to data streams. */
  /// LockableStreamMapPtr robochunkStreams_;
  LockableDataStreamMapPtr rosbag_streams_;
  /** \brief Location of the top-level run file */
  std::string filePath_;
  /** \brief Message structure containing run metadata */
  /// asrl::graph_msgs::Run msg_;
  vtr_messages::msg::GraphRun msg_;

  /** \brief Flag to indicate if this run can be saved normally */
  bool readOnly_;

  /** \brief Flag to indicate if edge/vertex files were loaded */
  bool wasLoaded_;

#if 0
  /** \brief Map for buffering Robochunk Messages to be inserted */
  std::map<std::string, std::queue<RCMsg>> streamBuffers_;

  /** \brief Lock for reading/writing to the streamBuffers_ */
  std::mutex streamBufferLock_;
#endif
  /** \brief The robot id, used for persistent ID lookup */
  IdType robotId_;

  friend class RCGraph;
  template <typename V, typename E, typename R>
  friend class Graph;
};

}  // namespace pose_graph
}  // namespace vtr

#include <vtr_pose_graph/index/rc_graph/rc_run.inl>
