// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// pybind11 bridge: Python episodes call LearnedStrategy::computeWaitTime in C++.

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "vtr_logging/logging_init.hpp"
#include "vtr_logging/configure.hpp"
#include "vtr_navigation/wait_strategy.hpp"
#include "vtr_tactic/types.hpp"

namespace py = pybind11;

using vtr::navigation::EdgeIdSet;
using vtr::navigation::LearnedStrategy;
using vtr::navigation::WaitDecision;
using vtr::navigation::WaitStrategyConfig;
using vtr::tactic::EdgeId;
using vtr::tactic::VertexId;

namespace {

void quietLogging() {
  static bool once = false;
  if (once) return;
  once = true;
  vtr::logging::configureLogging("", false);
  el::Configurations conf;
  conf.setToDefault();
  conf.setGlobally(el::ConfigurationType::Enabled, "false");
  el::Loggers::reconfigureAllLoggers(conf);
}

VertexId vid(int minor) { return VertexId(0, static_cast<uint32_t>(minor)); }

}  // namespace

/**
 * Persistent C++ wait engine for the Python sim hybrid.
 *
 * Call set_graph once (or when graph changes), sync_km / sync_stats / sync_memory
 * whenever Python state changes (typically each episode or each decision), then
 * compute_wait_time() which is wall-clock timed in C++.
 */
class WaitEngine {
 public:
  WaitEngine() { quietLogging(); }

  void configure(int W_grid_points, int T_grid_points, double speed_mps,
                 const std::map<std::string, double>& W_max_per_type) {
    cfg_.W_grid_points = W_grid_points;
    cfg_.T_grid_points = T_grid_points;
    cfg_.robot_speed_mps = speed_mps;
    cfg_.W_max_per_type = W_max_per_type;
    cfg_.type_weights.clear();
    for (const auto& kv : W_max_per_type) {
      cfg_.type_weights[kv.first] = 1.0;
    }
    cfg_.learned_data_dir.clear();
    cfg_.debug_plot_policy = false;
    cfg_.seed_samples.clear();
    strategy_ = std::make_unique<LearnedStrategy>(cfg_);
    wireGraph();
  }

  void set_graph(const std::vector<std::tuple<int, int, double>>& directed_edges) {
    neighbors_.clear();
    travel_.clear();
    for (const auto& trip : directed_edges) {
      const int u = std::get<0>(trip);
      const int v = std::get<1>(trip);
      const double tt = std::get<2>(trip);
      neighbors_[u].push_back(v);
      EdgeId e(vid(u), vid(v));
      travel_[e.hash()] = tt;
    }
    if (strategy_) wireGraph();
  }

  void sync_km(const std::vector<std::tuple<std::string, double, bool>>& samples) {
    ensureStrategy();
    strategy_->survivalModel()->clear();
    for (const auto& s : samples) {
      strategy_->survivalModel()->addSample(std::get<0>(s), std::get<1>(s),
                                            std::get<2>(s), /*episode=*/0);
    }
  }

  void sync_stats(int edges_traversed,
                  const std::vector<std::pair<std::string, int>>& type_episodes) {
    ensureStrategy();
    auto* stats = strategy_->obstacleStats();
    stats->clear();
    if (edges_traversed > 0) {
      stats->recordEdgeTraversals(edges_traversed);
    }
    for (const auto& kv : type_episodes) {
      for (int i = 0; i < kv.second; ++i) {
        stats->recordObstacleEpisode(kv.first);
      }
    }
  }

  void sync_memory(
      const std::vector<std::tuple<int, int, std::string, double, double>>& mems) {
    ensureStrategy();
    strategy_->resetMemory();
    for (const auto& m : mems) {
      EdgeId e(vid(std::get<0>(m)), vid(std::get<1>(m)));
      strategy_->seedMemoryForEdge(e, std::get<2>(m), std::get<3>(m), std::get<4>(m));
    }
  }

  py::dict compute_wait_time(const std::string& obs_type, int blocked_u, int blocked_v,
                             int current, int goal, double t_now, double obstacle_t_first) {
    ensureStrategy();
    EdgeIdSet blocked;
    blocked.insert(EdgeId(vid(blocked_u), vid(blocked_v)));

    const auto t0 = std::chrono::steady_clock::now();
    WaitDecision dec = strategy_->computeWaitTime(
        obs_type, blocked, vid(current), vid(goal), t_now, obstacle_t_first);
    const auto t1 = std::chrono::steady_clock::now();
    const double wall = std::chrono::duration<double>(t1 - t0).count();

    // computeWaitTime records the current obstacle into memory; strip it so the
    // next sync_memory from Python is authoritative.
    strategy_->clearMemoryForEdge(EdgeId(vid(blocked_u), vid(blocked_v)));

    py::dict out;
    out["W_star"] = dec.W_star;
    out["should_wait"] = dec.should_wait;
    out["wall_time_s"] = wall;
    out["speech"] = dec.speech;
    return out;
  }

 private:
  void ensureStrategy() {
    if (!strategy_) {
      throw std::runtime_error("WaitEngine: call configure() before use");
    }
  }

  void wireGraph() {
    if (!strategy_) return;
    strategy_->setGraphAccess(
        [this](const VertexId& v) -> std::vector<VertexId> {
          const int id = static_cast<int>(v.minorId());
          auto it = neighbors_.find(id);
          if (it == neighbors_.end()) return {};
          std::vector<VertexId> out;
          out.reserve(it->second.size());
          for (int n : it->second) out.push_back(vid(n));
          return out;
        },
        [this](const EdgeId& e) -> double {
          auto it = travel_.find(e.hash());
          if (it == travel_.end()) return 1e9;
          return it->second;
        });
  }

  WaitStrategyConfig cfg_;
  std::unique_ptr<LearnedStrategy> strategy_;
  std::unordered_map<int, std::vector<int>> neighbors_;
  std::unordered_map<size_t, double> travel_;
};

PYBIND11_MODULE(vtr_wait_strategy_py, m) {
  m.doc() = "C++ LearnedStrategy::computeWaitTime for Python OSCAR episodes";

  py::class_<WaitEngine>(m, "WaitEngine")
      .def(py::init<>())
      .def("configure", &WaitEngine::configure, py::arg("W_grid_points"),
           py::arg("T_grid_points"), py::arg("speed_mps"), py::arg("W_max_per_type"))
      .def("set_graph", &WaitEngine::set_graph, py::arg("directed_edges"))
      .def("sync_km", &WaitEngine::sync_km, py::arg("samples"))
      .def("sync_stats", &WaitEngine::sync_stats, py::arg("edges_traversed"),
           py::arg("type_episodes"))
      .def("sync_memory", &WaitEngine::sync_memory, py::arg("memories"))
      .def("compute_wait_time", &WaitEngine::compute_wait_time, py::arg("obs_type"),
           py::arg("blocked_u"), py::arg("blocked_v"), py::arg("current"), py::arg("goal"),
           py::arg("t_now"), py::arg("obstacle_t_first"));
}
