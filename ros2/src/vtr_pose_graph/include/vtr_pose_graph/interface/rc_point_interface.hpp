#pragma once

#include <map>

#include <vtr_common/utils/lockable.hpp>
#include <vtr_pose_graph/interface/rc_interface_types.hpp>

#if 0
#include <stdint.h>
#include <stdexcept>
#include <string>
#include <vector>

#include <asrl/messages/Utility.pb.h>
#include <google/protobuf/repeated_field.h>

#include <robochunk_msgs/TimeStamp.pb.h>
#include <robochunk/base/ChunkSerializer.hpp>
#include <robochunk/base/DataBubble.hpp>

#include <asrl/common/utils/ContainerTools.hpp>
#include <asrl/pose_graph/interface/RCInterfaceTypes.hpp>
#endif

namespace vtr {
namespace pose_graph {

class RCPointInterface {
 public:
  using PointMap = std::map<uint32_t, std::vector<uint64_t> >;
  using LockablePointMap = common::Lockable<PointMap>;

  using FieldMap = std::map<std::string, uint32_t>;
  using LockableFieldMap = common::Lockable<FieldMap>;
  using LockableFieldMapPtr = std::shared_ptr<LockableFieldMap>;

  // Structures to map between field ids and data streams.
  /// using StreamPtr = RobochunkIO::StreamPtr;
  /// using SerializerPtr = RobochunkIO::SerializerPtr;
  /// using StreamMap = std::map<uint32_t, RobochunkIO>;
  using DataStreamMap = std::map<uint32_t, RosBagIO>;
  using LockableDataStreamMap = common::Lockable<DataStreamMap>;
  using LockableDataStreamMapPtr = std::shared_ptr<LockableDataStreamMap>;

#if 0
  RCPointInterface()
      : pointNames_(LockableFieldMapPtr()), pointIndices_(PointMap()) {}

  RCPointInterface(
      const LockableFieldMapPtr& pointNames,
      const LockableStreamMapPtr& streamMap,
      const google::protobuf::RepeatedPtrField<asrl::graph_msgs::PointIndex>&
          pointIndices)
      : pointNames_(pointNames),
        streamMap_(streamMap),
        pointIndices_(PointMap()) {
    for (auto it = pointIndices.begin(); it != pointIndices.end(); ++it) {
      pointIndices_.locked().get().insert(std::make_pair(
          it->nameidx(),
          std::vector<uint64_t>(it->dataidx().begin(), it->dataidx().end())));
    }
  }

  RCPointInterface(const RCPointInterface&) = default;

  RCPointInterface(RCPointInterface&&) = default;

  RCPointInterface& operator=(const RCPointInterface&) = default;

  RCPointInterface& operator=(RCPointInterface&&) = default;

  void serializePoints(
      google::protobuf::RepeatedPtrField<asrl::graph_msgs::PointIndex>*
          pointIndices) const {
    pointIndices->Clear();
    pointIndices->Reserve(pointIndices_.locked().get().size());
    for (auto&& it : common::utils::getRefs(pointIndices_.locked().get())) {
      asrl::graph_msgs::PointIndex* tmpMsg = pointIndices->Add();
      tmpMsg->set_nameidx(it.get().first);

      for (auto&& jt : it.get().second) {
        tmpMsg->add_dataidx(jt);
      }
    }
  }

  //  void loadPoint(const std::string& pointName, uint64_t idx1 = 0, uint64_t
  //  idx2 = 0) { }
  void loadPoint(const std::string&, uint64_t, uint64_t) {
    throw std::logic_error(
        "[ERROR] RCPointInterface::loadPoint has not been implemented!");
  }

  //  void unloadPoint(const std::string& pointName, uint64_t idx1 = 0, uint64_t
  //  idx2 = 0) { }
  void unloadPoint(const std::string&, uint64_t, uint64_t) {
    throw std::logic_error(
        "[ERROR] RCPointInterface::unloadPoint has not been implemented!");
  }

  void unloadPointData() {
    throw std::logic_error(
        "[ERROR] RCPointInterface::unloadPointData has not been implemented!");
  }

 private:
  LockableFieldMapPtr pointNames_;

  LockableStreamMapPtr streamMap_;

  LockablePointMap pointIndices_;
#endif
};

}  // namespace pose_graph
}  // namespace vtr
