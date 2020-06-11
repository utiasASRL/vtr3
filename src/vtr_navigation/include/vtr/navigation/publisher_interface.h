#pragma once

#include <vtr/navigation/types.h>

//#include <asrl/pose_graph/index/RCGraph.hpp>
#include <asrl/pose_graph/path/LocalizationChain.hpp>

namespace vtr {
namespace navigation {

/** \brief Interface for a generic navigation state publisher
 */
class PublisherInterface {
 public:
  typedef asrl::pose_graph::RCGraph::TransformType TransformType;

  /** \brief Update localization messages for the path tracker
   */
  virtual void updateLocalization(const EdgeTransform &T_leaf_trunk,
                                  const EdgeTransform &T_root_trunk,
                                  const EdgeTransform &T_leaf_trunk_sensor,
                                  uint64_t stamp) = 0;

  /** \brief Set the path followed by the path tracker
   */
  virtual void publishPath(
      const asrl::pose_graph::LocalizationChain &chain) = 0;

  /** \brief Clear the path followed by the path tracker
   */
  virtual void clearPath() = 0;

  /** \brief Update robot messages for the UI
   */
  //  virtual void publishRobot(const pose_graph::LocalizationChain &chain,
  //  VertexId currentVertex) = 0;

  /** \brief Update robot messages for the UI
   */
  virtual void publishRobot(const Localization &persistentLoc,
                            uint64_t pathSeq = 0,
                            const Localization &targetLoc = Localization()) = 0;
};

}  // namespace navigation
}  // namespace vtr
