#pragma once

#include <vtr_common/utils/macros.hpp>
#include <vtr_navigation/tactics/basic_tactic.hpp>

namespace vtr {
namespace navigation {

using QueryCachePtr = std::shared_ptr<QueryCache>;
using MapCachePtr = std::shared_ptr<MapCache>;

class BasePipeline {
 public:
  PTR_TYPEDEFS(BasePipeline);

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
#if false
  virtual void makeKeyframeFromCandidate() {}
  virtual void processPetiole(QueryCachePtr query_data, MapCachePtr map_data,
                              bool first_frame) {}
#endif
  virtual const QueryCachePtr candidateQueryCache() const = 0;
  virtual const MapCachePtr candidateMapCache() const = 0;
#if 0
  virtual void computeT_0_q(QueryCachePtr q_data, MapCachePtr m_data) {}
#endif

  virtual void wait() {}

  VertexId addDanglingVertex(QueryCache& query_data) {
    auto live_id = tactic->addDanglingVertex(*query_data.stamp);
    query_data.live_id = live_id;
    return live_id;
  }
#if false
  VertexId addConnectedVertex(QueryCache& query_data,
                              const EdgeTransform& T_q_m) {
    auto live_id = tactic->addConnectedVertex(*query_data.stamp, T_q_m);
    return live_id;
  }
#endif
 protected:
  /// The tactic that owns this pipeline
  BasicTactic* tactic;
};

}  // namespace navigation
}  // namespace vtr
