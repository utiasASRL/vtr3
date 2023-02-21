// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file base_feature_extractor.cpp
 * \brief Source file for the ASRL vision package
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <list>

#include <vtr_logging/logging.hpp>
#include <vtr_vision/features/extractor/base_feature_extractor.hpp>

namespace vtr {
namespace vision {

using BFE = BaseFeatureExtractor;

Features BFE::extractFeatures(const Image &image) {
  Features features = extractFeatures(image.data);
  features.name = image.name;
  return features;
}

ChannelFeatures BFE::extractStereoFeatures(const Image &left,
                                           const Image &right) {
  ChannelFeatures features = extractStereoFeatures(left.data, right.data);
  if (features.cameras.size() != 2) return features;
  features.cameras[0].name = left.name;
  features.cameras[1].name = right.name;
  return features;
}

ChannelFeatures BFE::extractStereoFeaturesDisp(
    const Image &left, const Image &right, const Image &disp) {
  ChannelFeatures features =
      extractStereoFeaturesDisp(left.data, disp.data);
  if (features.cameras.size() != 2) return features;
  features.cameras[0].name = left.name;
  features.cameras[1].name = right.name;
  return features;
}

ChannelFeatures BFE::extractStereoFeaturesDispExtra(
    const Image &left, const Image &right, const Image &disp, 
    const Extra &extra) {
  ChannelFeatures features =
      extractStereoFeaturesDispExtra(left.data, disp.data, extra.keypoints,
                                     extra.descriptors, extra.scores);
  if (features.cameras.size() != 2) return features;
  features.cameras[0].name = left.name;
  features.cameras[1].name = right.name;
  return features;
}

ChannelExtra BFE::extractFeaturesExtra(const Image &left) {
  ChannelExtra extra = extractFeaturesExtra(left.data);
  if (extra.cameras.size() != 2) return extra;

  extra.cameras[0].name = left.name;
  return extra;
}

ChannelFeatures BFE::extractStereoFeatures(const ChannelImages &channel) {
  if (channel.cameras.size() != 2) {
    LOG(WARNING) << "Can't extract stereo features on "
                 << channel.cameras.size() << " images, will not fully match";
    return extractChannelFeatures(channel, false);
  }

  // If this is not an 8-bit grayscale image, then return an empty feature list.
  // if (channel.cameras[0].data.type() != CV_8UC1) {
  //   ChannelFeatures features;
  //   features.name = channel.name;
  //   return features;
  // }
  auto features = extractStereoFeatures(channel.cameras[0], channel.cameras[1]);
  features.name = channel.name;
  return features;
}

ChannelFeatures BFE::extractStereoFeaturesDisp(
    const ChannelImages &channel, const ChannelImages &channel_disp) {
  if (channel.cameras.size() != 2) {
    LOG(WARNING) << "Can't extract stereo features on "
                 << channel.cameras.size() << " images, will not fully match";
    return extractChannelFeatures(channel, false);
  }

  // If this is not an 8-bit grayscale image, then return an empty feature list.
  // if (channel.cameras[0].data.type() != CV_8UC1) {
  //   ChannelFeatures features;
  //   features.name = channel.name;
  //   return features;
  // }
  auto features = extractStereoFeaturesDisp(channel.cameras[0], 
                                            channel.cameras[1], 
                                            channel_disp.cameras[0]);
  features.name = channel.name;
  return features;
}

ChannelFeatures BFE::extractStereoFeaturesDispExtra(
    const ChannelImages &channel, const ChannelImages &channel_disp, 
    const ChannelExtra &channel_extra) {
  if (channel.cameras.size() != 2) {
    LOG(WARNING) << "Can't extract stereo features on "
                 << channel.cameras.size() << " images, will not fully match";
    return extractChannelFeatures(channel, false);
  }

  // If this is not an 8-bit grayscale image, then return an empty feature list.
  // if (channel.cameras[0].data.type() != CV_8UC1) {
  //   ChannelFeatures features;
  //   features.name = channel.name;
  //   return features;
  // }
  auto features = extractStereoFeaturesDispExtra(channel.cameras[0],
                                                 channel.cameras[1], 
                                                 channel_disp.cameras[0], 
                                                 channel_extra.cameras[0]);
  features.name = channel.name;
  return features;
}

ChannelExtra BFE::extractFeaturesExtra(const ChannelImages &channel) {
  auto extra = extractFeaturesExtra(channel.cameras[0]);
  extra.name = channel.name;

  return extra;
}

ChannelFeatures BFE::extractChannelFeatures(const ChannelImages &channel,
                                            bool fully_matched = false) {
  if (fully_matched && channel.cameras.size() == 2)
    return extractStereoFeatures(channel);

  ChannelFeatures features;
  features.name = channel.name;
  features.fully_matched = fully_matched && channel.cameras.size() == 2;

  // If this is not an 8-bit grayscale image, then return an empty feature list.
  if (!channel.cameras.empty() && channel.cameras[0].data.type() != CV_8UC1) {
    return features;
  }

  features.cameras.reserve(channel.cameras.size());
  std::list<std::future<Features>> futures;
  Features (My_t::*doit)(const Image &) = &My_t::extractFeatures;
  for (auto &cam : channel.cameras)
    futures.emplace_back(std::async(std::launch::async, doit, this, cam));
  for (auto &fut : futures) features.cameras.push_back(fut.get());

  return features;
}

ChannelFeatures BFE::extractChannelFeaturesDisp(
    const ChannelImages &channel, const ChannelImages &channel_disp, 
    bool fully_matched = false) {
  if (fully_matched && channel.cameras.size() == 2)
    // return extractStereoFeaturesDisp(channel, channel_disp);
    return extractStereoFeaturesDisp(channel, channel_disp);

  ChannelFeatures features;
  features.name = channel.name;
  features.fully_matched = fully_matched && channel.cameras.size() == 2;

  // If this is not an 8-bit grayscale image, then return an empty feature list.
  if (!channel.cameras.empty() && channel.cameras[0].data.type() != CV_8UC1) {
    return features;
  }

  features.cameras.reserve(channel.cameras.size());
  std::list<std::future<Features>> futures;
  Features (My_t::*doit)(const Image &) =
      &My_t::extractFeatures;
  for (auto &cam : channel.cameras)
    futures.emplace_back(std::async(std::launch::async, doit, this, cam));
  for (auto &fut : futures) features.cameras.push_back(fut.get());

  return features;
}

ChannelFeatures BFE::extractChannelFeaturesDispExtra(
    const ChannelImages &channel, const ChannelImages &channel_disp,
    const ChannelExtra &channel_extra, bool fully_matched = false) {
  if (fully_matched && channel.cameras.size() == 2)
    return extractStereoFeaturesDispExtra(channel, channel_disp, channel_extra);

  ChannelFeatures features;
  features.name = channel.name;
  features.fully_matched = fully_matched && channel.cameras.size() == 2;

  // If this is not an 8-bit grayscale image, then return an empty feature list.
  if (!channel.cameras.empty() && channel.cameras[0].data.type() != CV_8UC1) {
    return features;
  }

  features.cameras.reserve(channel.cameras.size());
  std::list<std::future<Features>> futures;
  Features (My_t::*doit)(const Image &) =
      &My_t::extractFeatures;
  for (auto &cam : channel.cameras)
    futures.emplace_back(std::async(std::launch::async, doit, this, cam));
  for (auto &fut : futures) features.cameras.push_back(fut.get());

  return features;
}

ChannelExtra BFE::extractChannelFeaturesExtra(const ChannelImages &channel) {
  ChannelExtra extra = extractFeaturesExtra(channel);

  return extra;
}

RigFeatures BFE::extractRigFeatures(const RigImages &rig,
                                    bool fully_matched = false) {
  RigFeatures features;
  features.name = rig.name;
  features.channels.reserve(rig.channels.size());

  std::list<std::future<ChannelFeatures>> futures;
  ChannelFeatures (My_t::*doit)(const ChannelImages &, bool) =
      &My_t::extractChannelFeatures;
  for (auto &chan : rig.channels)
    futures.emplace_back(
        std::async(std::launch::async, doit, this, chan, fully_matched));
  for (auto &fut : futures) features.channels.push_back(fut.get());

  return features;
}

}  // namespace vision
}  // namespace vtr
