

#include<vtr_vision/utils.hpp>

namespace vtr{
namespace vision {

RunIdSet fillRecommends(RunIdSet* recommends, const ScoredRids &distance_rids,
                        unsigned n) {
  RunIdSet new_recs;
  // go through the scored runs, from highest to lowest
  for (const auto &scored_rid : distance_rids) {
    // if we are supplementing existing recommendations
    if (recommends) {
      // insert the new run until the recommendation is large enough
      if (recommends->size() >= n) break;
      recommends->insert(scored_rid.second);
    }
    // recored the newly recommended runs until there are enough
    if (new_recs.size() >= n) break;
    new_recs.insert(scored_rid.second);
  }
  // return the newly recommended runs
  return new_recs;
}


}//namespace vision
}//namespace vtr