#pragma once

#include <vtr/navigation/pipelines/base_pipeline.h>

namespace vtr {

namespace navigation {

class IdlePipeline : public BasePipeline {
 public:
  IdlePipeline(BasicTactic* _tactic) : BasePipeline(_tactic) {}

  virtual void convertData(QueryCachePtr, MapCachePtr) {
    // auto converter = tactic->getImageFramer();
    // converter->run(q_data, m_data, tactic->poseGraph());
  }

  virtual KeyframeRequest processData(QueryCachePtr, MapCachePtr,
                                      bool first_frame = false) {
    // unused
    (void)first_frame;
    // \todo visualize only
    /*
        MapFrame mapframe;
        QueryFrame queryframe;
        vision::MatchList inliers;
        tactic->visualizeInliers(input_data,mapframe,queryframe,inliers,tactic->currentVertexID(),
       "VO Matches");
    */
    return KeyframeRequest::NO;
  }

  virtual void processKeyFrame(QueryCachePtr, MapCachePtr,
                               bool first_frame = false) {
    // unused
    (void)first_frame;
    // \todo visualize only
    /*
        MapFrame mapframe;
        QueryFrame queryframe;
        vision::MatchList inliers;
        tactic->visualizeInliers(input_data,mapframe,queryframe,inliers,tactic->currentVertexID(),
       "VO Matches");
    */
  }

  virtual void makeKeyFrame(QueryCachePtr, MapCachePtr,
                            bool first_frame = false) {
    // unused
    (void)first_frame;
    // \todo visualize only
    /*
        MapFrame mapframe;
        QueryFrame queryframe;
        vision::MatchList inliers;
        tactic->visualizeInliers(input_data,mapframe,queryframe,inliers,tactic->currentVertexID(),
       "VO Matches");
    */
  }

  // virtual void assessTerrain(QueryCachePtr q_data, MapCachePtr m_data, bool
  // ta_parallelization, std::future<void>& ta_thread_future) {}

  virtual const QueryCachePtr candidateQueryCache() const { return nullptr; }
  virtual const MapCachePtr candidateMapCache() const { return nullptr; }
};

}  // namespace navigation
}  // namespace vtr
