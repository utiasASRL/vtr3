
#include <chrono>
#include <list>

#include <vtr/vision/features/extractor/base_feature_extractor.h>

#include <asrl/common/logging.hpp>
#include <asrl/common/timing/SimpleTimer.hpp>

namespace vtr {
namespace vision {

using BFE = BaseFeatureExtractor;

asrl::vision::Features BFE::extractFeatures(const asrl::vision::Image &image) {
  asrl::vision::Features features = extractFeatures(image.data);
  features.name = image.name;
  return features;
}

asrl::vision::ChannelFeatures BFE::extractStereoFeatures(
    const asrl::vision::Image &left, const asrl::vision::Image &right) {
  asrl::vision::ChannelFeatures features =
      extractStereoFeatures(left.data, right.data);
  if (features.cameras.size() != 2) return features;
  features.cameras[0].name = left.name;
  features.cameras[1].name = right.name;
  return features;
}

asrl::vision::ChannelFeatures BFE::extractStereoFeatures(
    const asrl::vision::ChannelImages &channel) {
  if (channel.cameras.size() != 2) {
    LOG(WARNING) << "Can't extract stereo features on "
                 << channel.cameras.size() << " images, will not fully match";
    return extractChannelFeatures(channel, false);
  }

  // If this is not an 8-bit grayscale image, then return an empty feature list.
  if (channel.cameras[0].data.type() != CV_8UC1) {
    asrl::vision::ChannelFeatures features;
    features.name = channel.name;
    return features;
  }
  auto features = extractStereoFeatures(channel.cameras[0], channel.cameras[1]);
  features.name = channel.name;
  return features;
}

asrl::vision::ChannelFeatures BFE::extractChannelFeatures(
    const asrl::vision::ChannelImages &channel, bool fully_matched = false) {
  if (fully_matched && channel.cameras.size() == 2)
    return extractStereoFeatures(channel);

  asrl::vision::ChannelFeatures features;
  features.name = channel.name;
  features.fully_matched = fully_matched && channel.cameras.size() == 2;

  // If this is not an 8-bit grayscale image, then return an empty feature list.
  if (channel.cameras.size() > 0 && channel.cameras[0].data.type() != CV_8UC1) {
    return features;
  }

  features.cameras.reserve(channel.cameras.size());
  std::list<std::future<asrl::vision::Features>> futures;
  asrl::vision::Features (My_t::*doit)(const asrl::vision::Image &) =
      &My_t::extractFeatures;
  for (auto &cam : channel.cameras)
    futures.emplace_back(std::async(std::launch::async, doit, this, cam));
  for (auto &fut : futures) features.cameras.push_back(fut.get());

  return features;
}

asrl::vision::RigFeatures BFE::extractRigFeatures(
    const asrl::vision::RigImages &rig, bool fully_matched = false) {
  asrl::vision::RigFeatures features;
  features.name = rig.name;
  features.channels.reserve(rig.channels.size());

  std::list<std::future<asrl::vision::ChannelFeatures>> futures;
  asrl::vision::ChannelFeatures (My_t::*doit)(
      const asrl::vision::ChannelImages &, bool) =
      &My_t::extractChannelFeatures;
  for (auto &chan : rig.channels)
    futures.emplace_back(
        std::async(std::launch::async, doit, this, chan, fully_matched));
  for (auto &fut : futures) features.channels.push_back(fut.get());

  return features;
}

}  // namespace vision
}  // namespace vtr
