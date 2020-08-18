#pragma once

// #include <vtr/navigation/caches.h>
#include <vtr/navigation/tactics/basic_tactic.h>

#include <asrl/common/utils/CommonMacros.hpp>

namespace vtr {
namespace navigation {

class BasicTactic;  // TODO: need to remove this
using QueryCachePtr = std::shared_ptr<QueryCache>;
using MapCachePtr = std::shared_ptr<MapCache>;

class BasePipeline {
 public:
  PTR_TYPEDEFS(BasePipeline)

  BasePipeline(BasicTactic* _tactic) : tactic(_tactic) {}
  // Note: assumed to be ordered by increasing priority
  enum struct KeyframeRequest {
    NO = 0,
    IF_AVAILABLE = 1,
    YES = 2,
  };

  virtual void convertData(QueryCachePtr query_data, MapCachePtr map_data) = 0;
  virtual KeyframeRequest processData(QueryCachePtr query_data,
                                      MapCachePtr map_data,
                                      bool first_frame) = 0;
  virtual void makeKeyFrame(QueryCachePtr query_data, MapCachePtr map_data,
                            bool first_frame) = 0;
  virtual void processKeyFrame(QueryCachePtr query_data, MapCachePtr map_data,
                               bool first_frame) = 0;
  virtual void makeKeyframeFromCandidate() {}
  virtual void processPetiole(QueryCachePtr query_data, MapCachePtr map_data,
                              bool first_frame) {}
  virtual const QueryCachePtr candidateQueryCache() const = 0;
  virtual const MapCachePtr candidateMapCache() const = 0;
#if 0
  virtual void assessTerrain(QueryCachePtr q_data, MapCachePtr m_data,
                             bool ta_parallelization,
                             std::future<void>& ta_thread_future) = 0;
  virtual void computeT_0_q(QueryCachePtr q_data, MapCachePtr m_data) {}
#endif
  virtual void wait() {}

  VertexId addDanglingVertex(QueryCache& query_data) {
    auto live_id = tactic->addDanglingVertex(*query_data.stamp);
    query_data.live_id = live_id;
    return live_id;
  }

  VertexId addConnectedVertex(QueryCache& query_data,
                              const EdgeTransform& T_q_m) {
    auto live_id = tactic->addConnectedVertex(*query_data.stamp, T_q_m);
    return live_id;
  }

 protected:
  /// The tactic that owns this pipeline
  BasicTactic* tactic;
};

}  // namespace navigation
}  // namespace vtr
