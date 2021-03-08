#pragma once

#include <vector>

#include <vtr_common/timing/time_utils.hpp>
#include <vtr_pose_graph/evaluator/weight_evaluator.hpp>

namespace vtr {
namespace pose_graph {
namespace eval {

namespace Weight {

using common::timing::clock;
using common::timing::time_point;

struct TimeDeltaConfig {
  TimeDeltaConfig(const time_point& stamp = clock::now(),
                  std::chrono::hours utcOffset = std::chrono::hours(-5 * 3600),
                  double timeFalloff = 0.5, double daysFalloff = 0.25,
                  double maxContrib = 0.95, double smoothing = 0.0,
                  bool invert = false, uint32_t maxRuns = 5)
      : stamp_(stamp + utcOffset),
        utcOffset_(utcOffset),
        timeFalloff_(timeFalloff),
        daysFalloff_(daysFalloff),
        maxContrib_(maxContrib),
        smoothing_(smoothing),
        invert_(invert),
        maxRuns_(maxRuns) {
  }

  time_point stamp_;
  std::chrono::hours utcOffset_;
  double timeFalloff_;
  double daysFalloff_;
  double maxContrib_;
  double smoothing_;
  bool invert_;
  uint32_t maxRuns_;
};

/** \brief Evaluator for computing time difference costs */
DIRECT_EVAL(TimeDelta) {
 public:
  using decimal_hours = std::chrono::duration<double, std::ratio<3600>>;
  // using days = std::chrono::duration<common::timing::days> ;
  using time_t = date::time_of_day<common::timing::nanoseconds>;
  using date_t = date::year_month_day;
  using day_point = date::sys_days;
  using Config = TimeDeltaConfig;

  template <typename... Args>
  static std::shared_ptr<EvalBase<double>> MakeShared(Args && ... args) {
    return std::shared_ptr<EvalBase<double>>(
        new TimeDeltaDirect(std::forward<Args>(args)...));
  }

  DIRECT_PREAMBLE(TimeDelta);
  EVAL_DESTRUCTOR(TimeDelta, Direct) {
  }
  EVAL_CONSTRUCTOR(TimeDelta, Direct,
                   (const TimeDeltaConfig& config = TimeDeltaConfig()),
                   (config))
      : config_(config),
        currentTime_(common::timing::timePart(config.stamp_)),
        currentDate_(common::timing::datePart(config.stamp_)) {
  }

 protected:
  EVAL_COMPUTE_EDGE const {
    return std::min(this->computeVertex(this->graph_->at(e->to())),
                    this->computeVertex(this->graph_->at(e->from())));
  }

  EVAL_COMPUTE_VERTEX const {
    using namespace common::timing;

    auto adj = v->spatialNeighbours();
    adj.insert(v->id());
    std::set<double> costs;

    for (auto&& it : adj) {
      auto tp =
          toChrono(this->graph_->at(it)->keyFrameTime()) + config_.utcOffset_;
      auto hours = decimal_hours(timePart(tp).to_duration() -
                                 currentTime_.to_duration());
      double dt = std::abs(hours.count());
      int dd = common::timing::days(
                   (date::floor<days>(config_.stamp_) - date::floor<days>(tp)))
                   .count();

      // You can't be more than 12h away from any other time
      if (dt > 12.0)
        dt = 24.0 - dt;

      // Each run has p(localize) that falls off as a gaussian in time of day
      // and as an exponential in days elapsed
      costs.emplace(config_.maxContrib_ *
                    (std::exp(-0.5 * std::pow(dt * config_.timeFalloff_, 2) -
                              double(dd) * config_.daysFalloff_)));
    }

    unsigned int used = 0;
    double cost = 0.0;
    auto it = costs.rbegin();

    // The probabiliy that we do not localize is the product of 1 - p(localize)
    // for our best runs We do this in the log domain just in case we have small
    // probabilities
    while (it != costs.rend() && used < config_.maxRuns_) {
      cost += std::log(std::max(1.0 - *it, 1e-5));
      ++it;
      ++used;
    }

    if (config_.invert_) {
      // If in inverted mode, return the NLL of not localizing
      return -(1.0 - config_.smoothing_) *
                 std::log(std::min(std::exp(cost), 1.0 - 1e-5)) +
             config_.smoothing_;
    } else {
      // The probability that we localize against at least one run is 1 - p(no
      // localizations)
      return -(1.0 - config_.smoothing_) *
                 std::log(std::max(1.0 - std::exp(cost), 1e-5)) +
             config_.smoothing_;
    }
  }

  const TimeDeltaConfig config_;
  const time_t currentTime_;
  const date_t currentDate_;
};

CACHING_EVAL(TimeDelta) {
 public:
  CACHING_PREAMBLE(TimeDelta);
  EVAL_DESTRUCTOR(TimeDelta, Caching) {
  }
  EVAL_CONSTRUCTOR(TimeDelta, Caching,
                   (const TimeDeltaConfig& config = TimeDeltaConfig()),
                   (config))
      : DirectBase(config) {
  }

  template <typename... Args>
  static std::shared_ptr<EvalBase<double>> MakeShared(Args && ... args) {
    return std::shared_ptr<EvalBase<double>>(
        new TimeDeltaCaching(std::forward<Args>(args)...));
  }
};

WINDOWED_EVAL(TimeDelta) {
 public:
  WINDOWED_PREAMBLE(TimeDelta);
  EVAL_DESTRUCTOR(TimeDelta, Windowed) {
  }
  EVAL_CONSTRUCTOR(TimeDelta, Windowed,
                   (const TimeDeltaConfig& config = TimeDeltaConfig()),
                   (config))
      : DirectBase(config) {
  }
  EVAL_CONSTRUCTOR(TimeDelta, Windowed,
                   (const TimeDeltaConfig& config, size_t N), (config, N))
      : AbstractBase(N), DirectBase(config) {
  }

  template <typename... Args>
  static std::shared_ptr<EvalBase<double>> MakeShared(Args && ... args) {
    return std::shared_ptr<EvalBase<double>>(
        new TimeDeltaWindowed(std::forward<Args>(args)...));
  }
};

EVAL_TYPEDEFS(TimeDelta);

}  // namespace Weight

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr
