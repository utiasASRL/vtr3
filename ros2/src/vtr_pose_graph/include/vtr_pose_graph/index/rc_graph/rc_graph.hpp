#pragma once

#include <vtr_messages/msg/graph_run_list.hpp>
#include <vtr_messages/msg/time_stamp.hpp>
#include <vtr_pose_graph/index/graph.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph_base.hpp>
#include <vtr_pose_graph/index/rc_graph/types.hpp>

#if 0
#include <robochunk/base/DataInputStream.hpp>
#include <robochunk/base/DataOutputStream.hpp>

#include <asrl/messages/MapInfo.pb.h>
#include <asrl/messages/Run.pb.h>

#include <unordered_set>
#endif
namespace vtr {
namespace pose_graph {

// Virtual inheritance is necessary to ensure that relevant methods in GraphBase
// stay overridden
class RCGraph : public RCGraphBase, public Graph<RCVertex, RCEdge, RCRun> {
 public:
#if 0
  using RunFilter = std::unordered_set<RunIdType>;
  using SerializerPtr = std::shared_ptr<robochunk::base::ChunkSerializer>;
#endif

  using GraphType = Graph<RCVertex, RCEdge, RCRun>;
  using RType = RCGraphBase;

  using GraphType::Base;
  using GraphType::mtx_;

  using Base::edges_;
  using Base::graph_;
  using Base::id_;
  using Base::runs_;
  using Base::vertices_;

  PTR_TYPEDEFS(RCGraph)
  /**
   * \brief Interface to downcast base class pointers
   * \details This allows us to do DerivedPtrType = Type::Cast(BasePtrType)
   */
  //  PTR_DOWNCAST_OPS(RCGraph, Graph<RCVertex, RCEdge, RCRun>)

  /** \brief Pseudo constructor for making shared pointers */
  static Ptr MakeShared();
  static Ptr MakeShared(const std::string& filePath, const IdType& id);
#if 0
  static Ptr MakeShared(const std::string& filePath);
  static Ptr LoadOrCreate(const std::string& filePath,
                          const IdType& id = IdType(0));
#endif
  /** \brief Default constructor */
  RCGraph();
  /** \brief Construct an empty graph with an id and save location */
  RCGraph(const std::string& filePath, const IdType& id);
#if 0
  /** \brief Construct an graph, pointing to an index file */
  RCGraph(const std::string& filePath);
#endif
  /** \brief Copy and move operators */
  RCGraph(const RCGraph&) = default;
  RCGraph(RCGraph&& other);

  /** \brief Move assignment manually implemented due to virtual inheritance. */
  RCGraph& operator=(const RCGraph&) = default;
  RCGraph& operator=(RCGraph&& other);

  /** \brief Return a blank vertex(current run) with the next available Id */
  /// virtual VertexPtr addVertex(const robochunk::std_msgs::TimeStamp& time);
  virtual VertexPtr addVertex(const vtr_messages::msg::TimeStamp& time);
  /** \brief Return a blank vertex with the next available Id */
  /// virtual VertexPtr addVertex(const robochunk::std_msgs::TimeStamp& time,
  ///                             const RunIdType& runId);
  virtual VertexPtr addVertex(const vtr_messages::msg::TimeStamp& time,
                              const RunIdType& runId);
#if 0
  /** \brief Load the top-level index from file */
  void loadIndex();
  /** \brief Load the indexes of each run to inspect descriptions */
  void loadRunIndexes(const RunFilter& r = RunFilter());
  /** \brief Deep load runs and their vertex/edge data */
  void loadRuns(const RunFilter& r = RunFilter());
  /** \brief Deep load all levels of index data */
  void load(const RunFilter& r = RunFilter());

  /** \brief Save the top-level index to a working file */
  void saveWorkingIndex();
  /** \brief Save all modified runs to temporary files */
  void saveWorkingRuns();
  /** \brief Save all modifications to temporary files */
  void saveWorking();
#endif
  /** \brief Save the top-level index to file */
  void saveIndex();
  /** \brief Save all modified runs to file */
  void saveRuns(bool force = false);
  /** \brief Save everything to file */
  void save(bool force = false);

  /** \brief Get the file path of the graph index */
  std::string filePath() const { return filePath_; }
  /**
   * \brief Add a new run an increment the run id
   * \details This function is disabled for RCGraphs....
   */
  RunIdType addRun() override {
    std::stringstream ss;
    ss << "addRun(robotId) must be called for RCGraphs\n";
#if 0
       << el::base::debug::StackTrace();
#endif
    throw std::runtime_error(ss.str());
    return RunIdType(-1);
  }
  /** \brief Add a new run an increment the run id */
  virtual RunIdType addRun(IdType robotId, bool ephemeral = false,
                           bool extend = false, bool dosave = true);

  /** \brief Removes any temporary runs, if they exist */
  void removeEphemeralRuns();

  /** \brief registers a stream to a run. */
  void registerVertexStream(const RunIdType& run_id,
                            const std::string& stream_name,
                            bool points_to_data = true,
                            const RegisterMode& mode = RegisterMode::Create);
#if 0
  /** \brief Ensure correct vertex indices for a data stream */
  void reindexStream(const RunIdType& run_id, const std::string& stream_name,
                     const WindowType& wType = WindowType::Before);

  /** \brief Raw stream write access for data that isn't vertex-indexed */
  inline const SerializerPtr& writeStream(const RunIdType& run_id,
                                          const std::string& streamName) const {
    if (runs_ != nullptr && runs_->find(run_id) != runs_->end()) {
      return runs_->at(run_id)->writeStream(streamName);
    } else {
      LOG(ERROR) << "[RCGraphBase::readStream] Run " << run_id
                 << " was not in the run map.";
      throw std::runtime_error(
          "[RCGraph::writeStream] Attempted to access non-existent run");
    }
  }

  /** \brief Get the map display calibration */
  const asrl::graph_msgs::MapInfo& mapInfo() const;
  /** \brief Get the map display calibration */
  asrl::graph_msgs::MapInfo* mutableMapInfo();
  /** \brief Determine if a display map has been set for this graph */
  inline bool hasMap() const { return msg_.has_map(); }
  /** \brief Set the map display calibration */
  void setMapInfo(const asrl::graph_msgs::MapInfo& map);
  /** \brief Remove map information from a graph (USE CAREFULLY) */
  inline void clearMap() { msg_.clear_map(); }

  /**
   * \brief Removes any empty runs and associated folders from the graph.
   *        USE CAREFULLY, and only when you are shutting down the program.
   */
  void halt();
#endif
 protected:
  // Disable this function, since we need to know the timestamp
  VertexPtr addVertex() override {
    std::stringstream ss;
    ss << "Must provide timestamps for RCVertex";
#if 0
       << el::base::debug::StackTrace();
#endif
    throw std::runtime_error(ss.str());
    return VertexPtr();
  }

  // Disable this function, since we need to know the timestamp
  VertexPtr addVertex(const RunIdType&) override { return addVertex(); }
#if 0
  /** \brief Ensures that the vertex objects correctly reflect edge data */
  void linkEdgesInternal();

  /**
   * \brief Build map from persistent ids to existing vertex ids for fast lookup
   */
  void buildPersistentMap();
#endif
  std::string filePath_;

  /** \brief Ros message containing necessary information for a list of runs. */
  /// asrl::graph_msgs::RunList msg_;
  vtr_messages::msg::GraphRunList msg_;
};

#if 0
// TODO: Find a way to make explicit instantiation work in debug mode
#if !defined(RCGRAPH_NO_EXTERN) && defined(NDEBUG)
extern template class Graph<RCVertex, RCEdge, RunBase<RCVertex, RCEdge>>;

typedef Graph<RCVertex, RCEdge, RunBase<RCVertex, RCEdge>> __GraphRC;
EVAL_TYPED_DECLARE_EXTERN(double, __GraphRC)
EVAL_TYPED_DECLARE_EXTERN(bool, __GraphRC)

EVAL_TYPED_DECLARE_EXTERN(double, RCGraph)
EVAL_TYPED_DECLARE_EXTERN(bool, RCGraph)
#endif
#endif
}  // namespace pose_graph
}  // namespace vtr
