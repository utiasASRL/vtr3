#pragma once

#include <opencv2/core/core.hpp>

#include <asrl/navigation/caches.h>

#include <asrl/pose_graph/index/RCGraph.hpp>

namespace asrl {
namespace navigation {
namespace visualize {

/** \brief Sets up a map of RGB images used for visualization, based on all of
/// the rig images.
 */
std::map<std::string, cv::Mat> setupDisplayImages(QueryCache &qdata,
                                                  std::string suffix = "");

void showStereoMatches(std::mutex &vis_mtx, QueryCache &qdata,
                       std::string suffix = "");

/** \brief Adds visual features with depth coloring to the display images.
 */
void showRawFeatures(std::mutex &vis_mtx, QueryCache &qdata,
                     std::string suffix = "");

/** \brief Adds visual features with depth coloring to the display images.
 */
void showFeatures(std::mutex &vis_mtx, QueryCache &qdata,
                  std::string suffix = "");

void showMapFeatures(std::mutex &vis_mtx, QueryCache &qdata, MapCache &mdata,
                     std::string suffix = "");

void showMelMatches(std::mutex &vis_mtx, QueryCache &qdata, MapCache &mdata,
                    const pose_graph::RCGraph::ConstPtr &graph,
                    std::string suffix = "", int img_idx = 0);

void visualizeMEL(std::mutex &vis_mtx, QueryCache &qdata, MapCache &mdata,
                  const pose_graph::RCGraph::ConstPtr &graph,
                  std::string suffix = "", int img_idx = 0);
/** \brief Adds visual features with depth coloring to the display images.
 */
void showMatches(std::mutex &vis_mtx, QueryCache &qdata, MapCache &mdata,
                 std::vector<asrl::vision::RigMatches> &matches,
                 std::string suffix = "", bool plot_prediction = false);

}  // namespace visualize
}  // namespace navigation
}  // namespace asrl
