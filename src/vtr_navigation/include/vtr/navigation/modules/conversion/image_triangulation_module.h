#pragma once

#include <vtr/navigation/modules/base_module.h>

namespace asrl {
namespace navigation {

/** \brief Reject outliers and estimate a preliminary transform
 */
class ImageTriangulationModule : public BaseModule {
 public:
  static constexpr auto type_str_ = "image_triangulation";
  struct Config {
    // Triangulation Configuration
    bool visualize_features;
    bool visualize_stereo_features;
    float min_triangulation_depth;
    float max_triangulation_depth;
  };

  ImageTriangulationModule() : BaseModule{type_str_} {}

  /** \brief Given two frames and matches detects the inliers that fit the given
   * model, and provides an initial guess at transform T_q_m.
   */
  virtual void run(QueryCache &qdata, MapCache &,
                   const std::shared_ptr<const Graph> &);

  /** \brief Update the graph with optimized transforms
   */
  virtual void updateGraph(QueryCache &, MapCache &,
                           const std::shared_ptr<Graph> &, VertexId){};

  void setConfig(std::shared_ptr<Config> &config);

 protected:
  /** \brief Visualization implementation
   */
  virtual void visualizeImpl(QueryCache &qdata, MapCache &,
                             const std::shared_ptr<const Graph> &,
                             std::mutex &vis_mtx);

 private:
  /** \brief Algorithm Configuration
   */
  std::shared_ptr<Config> config_;
};

}  // namespace navigation
}  // namespace asrl
