#pragma once

#include <ros/ros.h>

#include <asrl/navigation/factories/module_factory.h>
#include <asrl/navigation/factories/ros_base_factory.h>

// #include <asrl/vision/features/extractor/ExtractorConfigs.hpp>

namespace asrl {
namespace navigation {

/** \brief make a module based on ros configuration
 */
class ROSModuleFactory : public ROSBaseFactory<BaseModule> {
 public:
  using base_t = ROSBaseFactory<BaseModule>;
  using mod_ptr = base_t::T_ptr;

  /** \brief constructed with ros param info
   * \param[in] nh the ros nodehandle with the params
   * \param[in] param_prefix the prefix (namespace) in the param path
   */
  ROSModuleFactory(nh_t *nh, const std::string &param_prefix)
      : base_t(nh, param_prefix) {}

 private:
  /** \brief constructs a module based on ros params
   * \param[in] type_str the type_str trait of the requested module
   */
  mod_ptr make_str(const std::string &type_str) const;

  //   /// @brief configures the module using rosparam
  //   /// @param[in] new_module pointer to the module
  //   /// @param[in] type_str the type_str trait of the requested module
  //   void configureModule(std::shared_ptr<BaseModule> &new_module,
  //                        const std::string &type_str) const;

  //   /// @brief configures the ORB detector using rosparam
  //   /// @param[in] config pointer to the base ORB config
  //   void configureORBDetector(vision::ORBConfiguration &config) const;

  // #if GPUSURF_ENABLED
  //   /// @brief configures the ORB detector using rosparam
  //   /// @param[in] config pointer to the base SURF config
  //   void configureSURFDetector(GpuSurfConfiguration &config) const;

  //   /// @brief configures the ORB detector using rosparam
  //   /// @param[in] config pointer to the base SURF config
  //   void configureSURFStereoDetector(GpuSurfStereoConfiguration &config)
  //   const;
  // #endif

  //   /// @brief configures the feature extractor module using rosparam
  //   /// @param[in] new_module pointer to the module
  //   void configureFeatureExtractor(std::shared_ptr<BaseModule> &new_module)
  //   const;

  //   /// @brief configures the image conversion / feature extractor combo
  //   module
  //   /// using rosparam
  //   /// @param[in] new_module pointer to the module
  //   void configureConversionExtractor(
  //       std::shared_ptr<BaseModule> &new_module) const;

  //   /// @brief configures the image converter module using rosparam
  //   /// @param[in] new_module pointer to the module
  //   void configureImageConverter(std::shared_ptr<BaseModule> &new_module)
  //   const;

  //   /// @brief configures the stereo disparity module using rosparam
  //   /// @param[in] new_module pointer to the module
  //   void configureCVStereoBM(std::shared_ptr<BaseModule> &new_module) const;

  //   /// @brief configures the stereo disparity module using rosparam
  //   /// @param[in] new_module pointer to the module
  //   void configureCVStereoSgbm(std::shared_ptr<BaseModule> &new_module)
  //   const;

  //   /// @brief configures the stereo disparity module using rosparam
  //   /// @param[in] new_module pointer to the module
  //   void configureCVGpuStereoBM(std::shared_ptr<BaseModule> &new_module)
  //   const;

  //   /// @brief configures the stereo disparity module using rosparam
  //   /// @param[in] new_module pointer to the module
  //   void configureElas(std::shared_ptr<BaseModule> &new_module) const;

  //   /// @brief configures the stereo disparity module using rosparam
  //   /// @param[in] new_module pointer to the module
  //   void configureCVReprojector(std::shared_ptr<BaseModule> &new_module)
  //   const;

  //   /// @brief configures the stereo disparity module using rosparam
  //   /// @param[in] new_module pointer to the module
  //   void configureCVGpuReprojector(std::shared_ptr<BaseModule> &new_module)
  //   const;

  //   /// @brief configures the image triangulation module using rosparam
  //   /// @param[in] new_module pointer to the module
  //   void configureImageTriangulator(
  //       std::shared_ptr<BaseModule> &new_module) const;

  //   /// @brief configures the mono image triangulation module using rosparam
  //   /// @param[in] new_module pointer to the module
  //   void configureMonoTriangulator(std::shared_ptr<BaseModule> &new_module)
  //   const;

  //   /// @brief configures the sequential image triangulation module using
  //   rosparam
  //   /// @param[in] new_module pointer to the module
  //   void configureSequentialTriangulator(
  //       std::shared_ptr<BaseModule> &new_module) const;

  //   /// @brief configures the base steam module using rosparam
  //   /// @param[in] new_module pointer to the module
  //   /// @param[in] config pointer to the base steam config
  //   void configureSteam(std::shared_ptr<SteamModule::Config> &config) const;

  //   /// @brief configures the stereo steam module using rosparam
  //   /// @param[in] new_module pointer to the module
  //   void configureKeyframeOptimization(
  //       std::shared_ptr<BaseModule> &new_module) const;

  //   /// @brief configures the stereo steam module using rosparam
  //   /// @param[in] new_module pointer to the module
  //   void configureWindowOptimization(
  //       std::shared_ptr<BaseModule> &new_module) const;

  //   /// @brief configures the opencv stereo matcher module using rosparam
  //   /// @param[in] new_module pointer to the module
  //   void configureOpenCVStereoMatcher(
  //       std::shared_ptr<BaseModule> &new_module) const;

  //   /// @brief configures the ASRL mono matcher module using rosparam
  //   /// @param[in] new_module pointer to the module
  //   void configureASRLMonoMatcher(std::shared_ptr<BaseModule> &new_module)
  //   const;

  //   /// @brief configures the ASRL stereo matcher module using rosparam
  //   /// @param[in] new_module pointer to the module
  //   void configureASRLStereoMatcher(
  //       std::shared_ptr<BaseModule> &new_module) const;

  //   /// @brief Base configures the base ransac module using rosparam
  //   /// @param[in] new_module pointer to the module
  //   /// @param[in] config pointer to the base ransac config
  //   void configureRANSAC(std::shared_ptr<RansacModule::Config> &config)
  //   const;

  //   /// @brief Stereo RANSAC configuration
  //   /// @param[in] new_module pointer to the module
  //   void configureStereoRANSAC(std::shared_ptr<BaseModule> &new_module)
  //   const;

  //   /// @brief Mono Initialization RANSAC configuration
  //   /// @param[in] new_module pointer to the module
  //   void configureInitMonoRANSAC(std::shared_ptr<BaseModule> &new_module)
  //   const;

  //   /// @brief Mono PnP RANSAC configuration
  //   /// @param[in] new_module pointer to the module
  //   void configureMonoRANSAC(std::shared_ptr<BaseModule> &new_module) const;

  //   // @brief Simple Vertex Creation Test configuration
  //   // @param[in] new_module pointer to the module.
  //   void configureSimpleVertexCreationTestModule(
  //       std::shared_ptr<BaseModule> &new_module) const;

  //   // @brief Lancaster Vertex Creation Test configuration
  //   // @param[in] new_module pointer to the module.
  //   void configureLancasterVertexCreationTestModule(
  //       std::shared_ptr<BaseModule> &new_module) const;

  //   // @brief Grizzly Gimbal Vertex Creation Test configuration
  //   // @param[in] new_module pointer to the module.
  //   void configureGimbalVertexCreationTestModule(
  //       std::shared_ptr<BaseModule> &new_module) const;

  //   void configureLandmarkRecallModule(
  //       std::shared_ptr<BaseModule> &new_module) const;

  //   void configureWindowedRecallModule(
  //       std::shared_ptr<BaseModule> &new_module) const;

  //   void configureSubMapExtraction(std::shared_ptr<BaseModule> &new_module)
  //   const;

  //   void configureLandmarkMigration(
  //       std::shared_ptr<BaseModule> &new_module) const;

  //   void configureMelMatcher(std::shared_ptr<BaseModule> &new_module) const;

  //   void configureMelRecog(std::shared_ptr<BaseModule> &new_module) const;

  //   void configureTodRecog(std::shared_ptr<BaseModule> &new_module) const;

  //   void configureCollabLandmarks(std::shared_ptr<BaseModule> &new_module)
  //   const;

  //   void configureRandomExperiences(
  //       std::shared_ptr<BaseModule> &new_module) const;

  //   void configureExperienceTriage(std::shared_ptr<BaseModule> &new_module)
  //   const;

  //   void configureResults(std::shared_ptr<BaseModule> &new_module) const;

  //   void configureMonoScaling(std::shared_ptr<BaseModule> &new_module) const;

  //   void configureQuickVORosPublisher(
  //       std::shared_ptr<BaseModule> &new_module) const;
  //   void configureRefinedVORosPublisher(
  //       std::shared_ptr<BaseModule> &new_module) const;
  //   void configureLocalizationRosPublisher(
  //       std::shared_ptr<BaseModule> &new_module) const;

  //   void configureUnderfootSeparate(
  //       std::shared_ptr<BaseModule> &new_module) const;

  //   void configureUnderfootAggregate(
  //       std::shared_ptr<BaseModule> &new_module) const;

  //   void configureLookaheadPatchGeneration(
  //       std::shared_ptr<BaseModule> &new_module) const;

  //   void configureTraining(std::shared_ptr<BaseModule> &new_module) const;

  //   void configureGpcTraining(std::shared_ptr<BaseModule> &new_module) const;

  //   void configureChangeDetection(std::shared_ptr<BaseModule> &new_module)
  //   const;

  //   void configureCDMaxMin(std::shared_ptr<BaseModule> &new_module) const;

  //   void configureCDMinMax(std::shared_ptr<BaseModule> &new_module) const;

  //   void configureCDGmm(std::shared_ptr<BaseModule> &new_module) const;

  //   void configureCDGpc(std::shared_ptr<BaseModule> &new_module) const;
};

}  // namespace navigation
}  // namespace asrl
