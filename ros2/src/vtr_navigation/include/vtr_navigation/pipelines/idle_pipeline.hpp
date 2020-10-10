#pragma once

#include <vtr_navigation/pipelines/base_pipeline.hpp>

namespace vtr {

namespace navigation {

class IdlePipeline : public BasePipeline {
 public:
  IdlePipeline(BasicTactic* _tactic) : BasePipeline(_tactic) {}

  void convertData(QueryCachePtr, MapCachePtr) override {
    // auto converter = tactic->getImageFramer();
    // converter->run(q_data, m_data, tactic->poseGraph());
  }

  KeyframeRequest processData(QueryCachePtr, MapCachePtr,
                              bool first_frame = false) override {
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
#if false
  void processKeyFrame(QueryCachePtr, MapCachePtr,
                       bool first_frame = false) override {
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

  void makeKeyFrame(QueryCachePtr, MapCachePtr,
                    bool first_frame = false) override {
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

  const QueryCachePtr candidateQueryCache() const override { return nullptr; }
  const MapCachePtr candidateMapCache() const override { return nullptr; }
#endif
};

}  // namespace navigation
}  // namespace vtr
