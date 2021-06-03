#pragma once

#include <opencv2/core/core.hpp>

#include <vtr_tactic/caches.hpp>
#include <vtr_tactic/types.hpp>

namespace vtr {
namespace tactic {
namespace visualize {

/**
 * \brief Sets up a map of RGB images used for visualization, based on all of
 * the rig images.
 */
std::map<std::string, cv::Mat> setupDisplayImages(QueryCache &qdata,
                                                  std::string suffix = "");

void showStereoMatches(std::mutex &vis_mtx, QueryCache &qdata,
                       std::string suffix = "");

/** \brief Adds visual features with depth coloring to the display images. */
void showRawFeatures(std::mutex &vis_mtx, QueryCache &qdata,
                     std::string suffix = "");

/** \brief Adds visual features with depth coloring to the display images. */
void showFeatures(std::mutex &vis_mtx, QueryCache &qdata,
                  std::string suffix = "");

void showMelMatches(std::mutex &vis_mtx, QueryCache &qdata, MapCache &mdata,
                    const pose_graph::RCGraph::ConstPtr &graph,
                    std::string suffix = "", int img_idx = 0);

/** \brief Adds visual features with depth coloring to the display images. */
void showMatches(std::mutex &vis_mtx, QueryCache &qdata, MapCache &mdata,
                 std::vector<vision::RigMatches> &matches,
                 std::string suffix = "", bool plot_prediction = false);
}  // namespace visualize
}  // namespace tactic
}  // namespace vtr
