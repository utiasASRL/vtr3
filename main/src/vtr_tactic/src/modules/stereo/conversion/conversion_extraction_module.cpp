#include <vtr_tactic/modules/stereo/conversion/conversion_extraction_module.hpp>
#include <vtr_vision/features/extractor/feature_extractor_factory.hpp>
#include <vtr_vision/image_conversions.hpp>
#include <vtr_vision/types.hpp>

namespace vtr {
namespace tactic {

void ConversionExtractionModule::setConfig(std::shared_ptr<Config> &config) {
  config_ = config;

  extractor_ =
      vision::FeatureExtractorFactory::createExtractor(config_->feature_type);
  if (config_->feature_type == "ASRL_GPU_SURF") {
#if GPUSURF_ENABLED
    vision::GpuSurfFeatureExtractor *dextractor =
        dynamic_cast<vision::GpuSurfFeatureExtractor *>(extractor_.get());
    dextractor->initialize(config_->gpu_surf_params);
    dextractor->initialize(config_->gpu_surf_stereo_params);
#else
    LOG(ERROR) << "GPU SURF isn't enabled!";
#endif
  } else if (config_->feature_type == "OPENCV_ORB") {
    vision::OrbFeatureExtractor *dextractor =
        dynamic_cast<vision::OrbFeatureExtractor *>(extractor_.get());
    dextractor->initialize(config_->opencv_orb_params);
  } else {
    LOG(ERROR) << "Couldn't determine feature type!";
  }
}

void ConversionExtractionModule::runImpl(QueryCache &qdata, MapCache &,
                                         const Graph::ConstPtr &) {
  // check if the required data is in this cache
  if (!qdata.rig_images.is_valid() || !qdata.rig_calibrations.is_valid())
    return;

  // Inputs, images
  auto &rigs = *qdata.rig_images;
  // Outputs, frames
  auto &rig_feature_list = qdata.rig_features.fallback();
  if (extractor_ == nullptr) {
    LOG(ERROR) << " Our extractor is null!";
    return;
  }
  for (auto &rig : rigs) {
    rig_feature_list->emplace_back(vision::RigFeatures());
    auto num_input_channels = rig.channels.size();

    auto &rig_features = rig_feature_list->back();
    rig_features.name = rig.name;
    vision::ChannelFeatures (vision::BaseFeatureExtractor::*doit)(
        const vision::ChannelImages &, bool) =
        &vision::BaseFeatureExtractor::extractChannelFeatures;
    for (unsigned channel_idx = 0; channel_idx < num_input_channels;
         ++channel_idx) {
      auto cc_weight_idx = 0;
      std::vector<std::future<vision::ChannelFeatures>> feature_futures;
      // extract features on this channel. The extractor config selects if the
      // channel requires feature extraction, otherwise it inserts an empty set
      // of channel features
      feature_futures.emplace_back(std::async(std::launch::async, doit,
                                              extractor_.get(),
                                              rig.channels[channel_idx], true));
      // make the appropriate conversions.
      for (unsigned conversion_idx = 0;
           conversion_idx < config_->conversions.size(); ++conversion_idx) {
        const auto &input_channel = rig.channels[channel_idx];
        const auto &conversion = vision::StringToImageConversion(
            config_->conversions[conversion_idx]);
        // convert
        if (conversion == vision::ImageConversion::RGB_TO_GRAYSCALE) {
          rig.channels.emplace_back(vision::RGB2Grayscale(input_channel));
        } else if (conversion ==
                   vision::ImageConversion::RGB_TO_COLOR_CONSTANT) {
          // move the new channel onto the rig.
          rig.channels.emplace_back(vision::RGB2ColorConstant(
              rig.channels[channel_idx],
              config_->color_constant_weights[cc_weight_idx],
              config_->color_constant_histogram_equalization));
          cc_weight_idx++;
        } else if (conversion == vision::ImageConversion::UNKNOWN) {
          throw std::runtime_error("ERROR: Image conversion " +
                                   config_->conversions[conversion_idx] +
                                   " unknown!");
        }

        // extract
        feature_futures.emplace_back(std::async(std::launch::async, doit,
                                                extractor_.get(),
                                                rig.channels.back(), true));
      }  // finish the conversions

      // get the futures.
      for (auto &future : feature_futures) {
        rig_features.channels.emplace_back(future.get());
      }
    }
  }
}

void ConversionExtractionModule::visualizeImpl(QueryCache &qdata, MapCache &,
                                               const Graph::ConstPtr &,
                                               std::mutex &vis_mtx) {
  if (config_->visualize_raw_features)  // check if visualization is enabled
    visualize::showRawFeatures(vis_mtx, qdata, " raw features");
}

}  // namespace tactic
}  // namespace vtr
