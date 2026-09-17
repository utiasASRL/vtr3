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
#include "vtr_navigation/sparrow_contract.hpp"
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

  // ---- Corridor contraction (paper Sec. IV; handoff Sec. 2) ---------------
  // Exposed so the C++ contraction can be cross-checked against
  // vtr3_sim/pomcp/contract.py on the real taught graph, vertex by vertex,
  // rather than only against the two summary counts.
  m.def(
      "build_macro_plan",
      [](const std::map<uint64_t, std::vector<uint64_t>>& neighbors,
         const std::map<std::pair<uint64_t, uint64_t>, double>& travel_time,
         uint64_t goal, uint64_t root, const std::set<uint64_t>& extra_nodes,
         double max_macro_len_s) {
        namespace sp = vtr::navigation::sparrow;
        sp::GraphContext micro(neighbors, travel_time, goal);
        const auto plan =
            sp::buildMacroPlan(micro, root, extra_nodes, max_macro_len_s);

        // (u, v, first_hop, n_micro, travel_time, chain) per macro-edge,
        // keyed by the canonical edge.
        py::dict macros;
        for (const auto& kv : plan.macros) {
          const auto& m2 = kv.second;
          macros[py::make_tuple(kv.first.first, kv.first.second)] =
              py::make_tuple(m2.u, m2.v, m2.first_hop, m2.n_micro(),
                             m2.travel_time, m2.chain);
        }
        py::dict out;
        out["macros"] = macros;
        out["nodes"] = plan.context.neighbors.size();
        out["corridors"] = plan.macros.size();
        std::vector<uint64_t> node_list;
        for (const auto& kv : plan.context.neighbors)
          node_list.push_back(kv.first);
        out["node_list"] = node_list;
        return out;
      },
      py::arg("neighbors"), py::arg("travel_time"), py::arg("goal"),
      py::arg("root"), py::arg("extra_nodes") = std::set<uint64_t>{},
      py::arg("max_macro_len_s") = 0.0);

  m.def("parse_corner_spec", [](const std::string& spec) {
    return vtr::navigation::sparrow::parseCornerSpec(spec);
  });

  m.def("macro_p_block", [](double p_micro, size_t n_micro) {
    return vtr::navigation::sparrow::macroPBlock(p_micro, n_micro);
  });

  // Per-class MaxWait grids (handoff sec. 1.8): an edge the robot has PAID to
  // Observe gets its class's own grid; everything else gets the shared one.
  m.def(
      "valid_actions",
      [](uint64_t vertex, const std::map<std::pair<uint64_t, uint64_t>, bool>& st,
         const std::vector<uint64_t>& nbrs,
         const std::set<std::pair<uint64_t, uint64_t>>& classified,
         const std::vector<double>& wait_set, bool allow_observe,
         const std::map<std::string, std::vector<double>>& by_class,
         const std::map<std::pair<uint64_t, uint64_t>, std::string>& classes) {
        namespace sp = vtr::navigation::sparrow;
        const auto acts = sp::valid_actions(
            vertex, st, nbrs, classified, wait_set, allow_observe,
            by_class.empty() ? nullptr : &by_class,
            classes.empty() ? nullptr : &classes);
        // (kind, edge, W): kind 0=TRAVERSE 1=MAXWAIT 2=OBSERVE
        std::vector<std::tuple<int, std::pair<uint64_t, uint64_t>, double>> out;
        for (const auto& a : acts) {
          out.emplace_back(static_cast<int>(a.kind), a.edge, a.W);
        }
        return out;
      },
      py::arg("vertex"), py::arg("statuses"), py::arg("neighbors"),
      py::arg("classified"), py::arg("wait_set"), py::arg("allow_observe") = true,
      py::arg("wait_set_by_class") = std::map<std::string, std::vector<double>>{},
      py::arg("edge_classes") =
          std::map<std::pair<uint64_t, uint64_t>, std::string>{});

  // Contracted occupancy: exercises the REAL ParticleBelief::setEdgeWeights
  // and p_edge path, not a reimplementation of the arithmetic. Given each
  // macro-edge's micro-edge count, returns the occupancy the belief will use.
  m.def(
      "belief_edge_occupancy",
      [](const std::map<std::pair<uint64_t, uint64_t>, size_t>& n_micro,
         double p_block) {
        namespace sp = vtr::navigation::sparrow;
        sp::SparrowModel model;
        model.p_block = p_block;
        sp::GraphContext ctx;
        sp::ParticleBelief belief(&ctx, &model, 1, false, 1);

        // w_e = macroPBlock(p, n) / p  (contract.py's _macro_view weights)
        std::map<std::pair<uint64_t, uint64_t>, double> raw;
        for (const auto& kv : n_micro) {
          raw[kv.first] = (p_block > 0.0)
                              ? sp::macroPBlock(p_block, kv.second) / p_block
                              : static_cast<double>(kv.second);
        }
        belief.setEdgeWeights(raw);

        std::map<std::pair<uint64_t, uint64_t>, double> out;
        for (const auto& kv : n_micro) out[kv.first] = belief.p_edge(kv.first);
        return out;
      },
      py::arg("n_micro"), py::arg("p_block"));

  // Corridor sight: a macro-edge is blocked iff ANY of its micro-edges is,
  // free only when EVERY one was observed free, unknown otherwise. While
  // driving, only the part of the chain ahead of the robot counts.
  m.def(
      "corridor_status",
      [](const std::vector<uint64_t>& chain,
         const std::map<std::pair<uint64_t, uint64_t>, int>& micro) {
        namespace sp = vtr::navigation::sparrow;
        return sp::corridorStatus(chain, [&](uint64_t a, uint64_t b) {
          auto it = micro.find(sp::canonical_edge(a, b));
          return (it == micro.end()) ? sp::kEdgeUnknown : it->second;
        });
      },
      py::arg("chain"), py::arg("micro_status"));

  m.def(
      "macro_chain_from",
      [](uint64_t u, uint64_t v, const std::vector<uint64_t>& chain,
         uint64_t from, bool remaining, uint64_t robot_at) {
        namespace sp = vtr::navigation::sparrow;
        sp::MacroEdge m2;
        m2.u = u;
        m2.v = v;
        m2.chain = chain;
        m2.first_hop = chain.empty() ? v : chain.front();
        return remaining ? sp::remainingChainFrom(m2, from, robot_at)
                         : sp::macroChainFrom(m2, from);
      },
      py::arg("u"), py::arg("v"), py::arg("chain"), py::arg("from_vertex"),
      py::arg("remaining") = false, py::arg("robot_at") = 0);

  // Marthi B-set (handoff-adjacent; ports runner._blocked_vertex_set). The
  // model is built with km = nullptr, so every class falls back to the
  // exponential prior and the survival weighting is analytic - which is what
  // makes the parity check closed-form rather than dependent on a KM bridge.
  m.def(
      "blocked_vertex_set",
      [](const std::map<std::pair<uint64_t, uint64_t>, int>& statuses,
         // edge -> (blocked, t_obs, age_at_obs, label)
         const std::map<std::pair<uint64_t, uint64_t>,
                        std::tuple<bool, double, double, std::string>>& memory,
         double time, double p_block,
         const std::vector<std::string>& class_names,
         const std::vector<double>& class_probs, double prior_residual_mean) {
        namespace sp = vtr::navigation::sparrow;
        sp::SparrowModel model;
        model.class_names = class_names;
        model.class_probs = class_probs;
        model.p_block = p_block;
        model.prior_residual_mean = prior_residual_mean;
        model.km = nullptr;

        sp::LocalObservation local;
        local.time = time;
        local.statuses = statuses;
        for (const auto& kv : memory) {
          sp::EdgeMemoryRec rec;
          rec.blocked = std::get<0>(kv.second);
          rec.t_obs = std::get<1>(kv.second);
          rec.age_at_obs = std::get<2>(kv.second);
          rec.label = std::get<3>(kv.second);
          local.memory[kv.first] = rec;
        }
        const auto bs = sp::blockedVertexSet(local, model);

        py::dict out;
        std::vector<std::pair<uint64_t, uint64_t>> edges(bs.edges.begin(),
                                                         bs.edges.end());
        std::vector<uint64_t> nodes(bs.nodes.begin(), bs.nodes.end());
        out["edges"] = edges;
        out["nodes"] = nodes;
        return out;
      },
      py::arg("statuses"), py::arg("memory"), py::arg("time"),
      py::arg("p_block"), py::arg("class_names"), py::arg("class_probs"),
      py::arg("prior_residual_mean") = 60.0);

  m.def(
      "memory_alive_prob",
      [](bool blocked, double t_obs, double age_at_obs,
         const std::string& label, double t_now,
         const std::vector<std::string>& class_names,
         const std::vector<double>& class_probs, double prior_residual_mean) {
        namespace sp = vtr::navigation::sparrow;
        sp::SparrowModel model;
        model.class_names = class_names;
        model.class_probs = class_probs;
        model.prior_residual_mean = prior_residual_mean;
        model.km = nullptr;
        sp::EdgeMemoryRec rec;
        rec.blocked = blocked;
        rec.t_obs = t_obs;
        rec.age_at_obs = age_at_obs;
        rec.label = label;
        return sp::memoryAliveProb(rec, t_now, model);
      },
      py::arg("blocked"), py::arg("t_obs"), py::arg("age_at_obs"),
      py::arg("label"), py::arg("t_now"), py::arg("class_names"),
      py::arg("class_probs"), py::arg("prior_residual_mean") = 60.0);
}
