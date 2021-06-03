#pragma once

#include <vtr_common/timing/time_utils.hpp>
#include <vtr_messages/msg/exp_recog_status.hpp>
#include <vtr_tactic/modules/base_module.hpp>

namespace vtr {
namespace tactic {

/**
 * \brief Recommend experiences based on time of day.
 * \details
 * Recommend experiences based on time of day.
 * requires:
 *   qdata.[live_id]
 *   mdata.[localization_map, *recommended_experience]
 * outputs:
 *   qdata.[]
 *   mdata.[recommended_experience]
 */
class TodRecognitionModule : public BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "timeofday_recognition";
  using time_point = common::timing::time_point;

  /** \brief Collection of config parameters */
  struct Config {
    /// Enables debugging logs.
    bool verbose = true;

    // INTERFACING //
    /// The number of experiences (including the privileged)
    /// that we should be recommended for localization.
    int num_exp = 4;
    /// Whether we should broadcast our recommendations (are we enabled).
    bool in_the_loop = false;

    // PARAMETERS //
    /// The weight to convert time-of-day difference to a distance
    float time_of_day_weight = 1.f;
    /// The weight to convert total time difference to a distance
    float total_time_weight = 1.f / 24.f;
  };

  TodRecognitionModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()) {}

  /**
   * \brief Sets the module's configuration.
   * \param config the input configuration.
   */
  void setConfig(std::shared_ptr<Config> &config) { config_ = config; }

 private:
  /** \brief \todo */
  void runImpl(QueryCache &qdata, MapCache &mdata,
               const Graph::ConstPtr &graph) override;

  /** \brief \todo */
  void updateGraphImpl(QueryCache &qdata, MapCache & /*mdata*/,
                       const Graph::Ptr &graph, VertexId vid) override;

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;

  /** \brief The status message to save to the graph */
  vtr_messages::msg::ExpRecogStatus status_msg_;
};

/**
 * \brief Compute the time/time-of-day distance for the experiences in the
 * submap based on distance from a query point in time.
 * \param query_tp The query time point
 * \param submap The submap to precess
 * \param config COnfiguration (for weights)
 * \return The scored experiences
 */
ScoredRids scoreExperiences(const TodRecognitionModule::time_point &query_tp,
                            const pose_graph::RCGraphBase &submap,
                            const TodRecognitionModule::Config &config);

}  // namespace tactic
}  // namespace vtr
