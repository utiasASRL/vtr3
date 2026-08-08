// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Offline scalability microbench for LearnedStrategy::computeWaitTime.
// Synthetic graphs match Python vtr3_sim.graph_loader._create_grid_graph
// used by configs/scalability_v*_k*.yaml.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "vtr_logging/logging_init.hpp"
#include "vtr_logging/configure.hpp"
#include "vtr_navigation/wait_strategy.hpp"
#include "vtr_tactic/types.hpp"

using vtr::navigation::EdgeIdSet;
using vtr::navigation::LearnedStrategy;
using vtr::navigation::WaitDecision;
using vtr::navigation::WaitStrategyConfig;
using vtr::tactic::EdgeId;
using vtr::tactic::VertexId;

namespace {

constexpr double kSegmentLengthM = 0.3;
constexpr double kSpeedMps = 0.95;
constexpr double kPBlock = 0.08;
constexpr int kSeedUncensored = 30;
constexpr int kWGridPoints = 300;
constexpr int kTGridPoints = 300;
constexpr int kDefaultTrials = 50;
constexpr int kWarmup = 2;
constexpr uint32_t kRngSeed = 7;

const std::vector<int> kTargetVertices = {50, 100, 250, 500, 1000};
const std::map<int, int> kGridNForTarget = {
    {50, 7}, {100, 10}, {250, 16}, {500, 22}, {1000, 32}};
const std::vector<int> kNumTypesLevels = {4, 8, 16};

struct GridGraph {
  int n = 0;
  double travel_time = 0.0;
  std::unordered_map<uint64_t, std::vector<VertexId>> neighbors;
  std::unordered_map<size_t, double> travel_times;  // EdgeId::hash()
};

uint64_t vidKey(const VertexId& v) {
  return (static_cast<uint64_t>(v.majorId()) << 32) |
         static_cast<uint64_t>(v.minorId());
}

VertexId toVertex(int n, int i, int j) {
  return VertexId(0, static_cast<uint32_t>(i * n + j));
}

/**
 * Exact port of Python _create_grid_graph:
 * - nodes (i,j), i,j in [0,n-1]
 * - bi-directed orthogonal edges only
 * - edge distance = L, travel_time = L / speed
 */
GridGraph buildPythonParityGrid(int n, double L, double speed) {
  if (n < 2) throw std::runtime_error("grid_n must be >= 2");
  if (speed <= 0.0) throw std::runtime_error("speed must be positive");

  GridGraph g;
  g.n = n;
  g.travel_time = L / speed;

  auto add_edge = [&](const VertexId& u, const VertexId& v) {
    g.neighbors[vidKey(u)].push_back(v);
    EdgeId e(u, v);
    g.travel_times[e.hash()] = g.travel_time;
  };

  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n - 1; ++j) {
      VertexId u = toVertex(n, i, j);
      VertexId v = toVertex(n, i, j + 1);
      add_edge(u, v);
      add_edge(v, u);
    }
  }
  for (int i = 0; i < n - 1; ++i) {
    for (int j = 0; j < n; ++j) {
      VertexId u = toVertex(n, i, j);
      VertexId v = toVertex(n, i + 1, j);
      add_edge(u, v);
      add_edge(v, u);
    }
  }

  const int expected_v = n * n;
  const int expected_e = 4 * n * (n - 1);  // directed
  int actual_e = 0;
  for (const auto& kv : g.neighbors) actual_e += static_cast<int>(kv.second.size());
  if (static_cast<int>(g.neighbors.size()) != expected_v) {
    throw std::runtime_error("grid parity: |V| mismatch");
  }
  if (actual_e != expected_e) {
    std::ostringstream oss;
    oss << "grid parity: |E| mismatch got " << actual_e << " expected " << expected_e;
    throw std::runtime_error(oss.str());
  }
  return g;
}

void assertGridInvariants(const GridGraph& g) {
  const int n = g.n;
  const int hops = 2 * (n - 1);
  const double expected_tt = kSegmentLengthM / kSpeedMps;
  if (std::abs(g.travel_time - expected_tt) > 1e-12) {
    throw std::runtime_error("grid parity: travel time mismatch");
  }
  // Corner-to-corner hop length on the lattice.
  if (hops <= 0) throw std::runtime_error("grid parity: bad hop length");
  std::cout << "  grid_n=" << n << " |V|=" << (n * n)
            << " |E|=" << (4 * n * (n - 1))
            << " tt=" << g.travel_time << "s"
            << " start→goal hops=" << hops << "\n";
}

std::vector<double> sampleLognormal(double mu, double sigma, int n, std::mt19937& rng) {
  std::lognormal_distribution<double> dist(mu, sigma);
  std::vector<double> out;
  out.reserve(n);
  for (int i = 0; i < n; ++i) out.push_back(dist(rng));
  return out;
}

WaitStrategyConfig makeConfig(int n_types, std::mt19937& rng) {
  WaitStrategyConfig cfg;
  cfg.W_grid_points = kWGridPoints;
  cfg.T_grid_points = kTGridPoints;
  cfg.robot_speed_mps = kSpeedMps;
  cfg.debug_plot_policy = false;
  cfg.learned_data_dir.clear();  // in-memory only

  // Match Python make_obstacle_types: mu linspace 1.5→5.2, sigma 0.45→0.75
  for (int i = 0; i < n_types; ++i) {
    const double t = (n_types == 1) ? 0.0 : static_cast<double>(i) / (n_types - 1);
    const double mu = 1.5 + t * (5.2 - 1.5);
    const double sigma = 0.45 + t * (0.75 - 0.45);
    const double mean_d = std::exp(mu + 0.5 * sigma * sigma);
    const double w_max = std::min(1000.0, std::max(200.0, 3.0 * mean_d));

    std::ostringstream name;
    name << "cls_" << std::setw(2) << std::setfill('0') << i;
    const std::string type = name.str();

    cfg.W_max_per_type[type] = w_max;
    cfg.type_weights[type] = 1.0;
    cfg.seed_samples[type] = sampleLognormal(mu, sigma, kSeedUncensored, rng);
  }
  return cfg;
}

void seedPBlock(LearnedStrategy& strategy, int n_types) {
  // p_block = episodes / edges_traversed ≈ 0.08
  constexpr int edges = 10000;
  const int episodes = static_cast<int>(std::lround(kPBlock * edges));
  strategy.obstacleStats()->recordEdgeTraversals(edges);
  for (int e = 0; e < episodes; ++e) {
    std::ostringstream name;
    name << "cls_" << std::setw(2) << std::setfill('0') << (e % n_types);
    strategy.obstacleStats()->recordObstacleEpisode(name.str());
  }
}

struct BenchRow {
  int target_vertices = 0;
  int actual_nodes = 0;
  int grid_n = 0;
  int n_types = 0;
  double decision_time_s = 0.0;
  double W_star = 0.0;
};

std::vector<BenchRow> runCell(int target_v, int n_types, int trials, std::mt19937& rng) {
  const int grid_n = kGridNForTarget.at(target_v);
  auto graph = buildPythonParityGrid(grid_n, kSegmentLengthM, kSpeedMps);
  assertGridInvariants(graph);

  auto neighbors_fn = [&graph](const VertexId& v) -> std::vector<VertexId> {
    auto it = graph.neighbors.find(vidKey(v));
    if (it == graph.neighbors.end()) return {};
    return it->second;
  };
  auto travel_fn = [&graph](const EdgeId& e) -> double {
    auto it = graph.travel_times.find(e.hash());
    if (it == graph.travel_times.end()) return 1e9;
    return it->second;
  };

  auto cfg = makeConfig(n_types, rng);
  LearnedStrategy strategy(cfg);
  seedPBlock(strategy, n_types);
  strategy.setGraphAccess(neighbors_fn, travel_fn);

  const VertexId start = toVertex(grid_n, 0, 0);
  const VertexId goal = toVertex(grid_n, grid_n - 1, grid_n - 1);
  // First edge on a shortest path: (0,0) → (0,1)
  const VertexId next = toVertex(grid_n, 0, 1);
  EdgeIdSet blocked{EdgeId(start, next)};
  const std::string obs_type = "cls_00";
  const double t_now = 0.0;
  const double t_first = 0.0;

  // Warmup
  for (int i = 0; i < kWarmup; ++i) {
    strategy.resetMemory();
    (void)strategy.computeWaitTime(obs_type, blocked, start, goal, t_now, t_first);
  }

  std::vector<BenchRow> rows;
  rows.reserve(trials);
  for (int t = 0; t < trials; ++t) {
    strategy.resetMemory();
    const auto t0 = std::chrono::steady_clock::now();
    WaitDecision dec =
        strategy.computeWaitTime(obs_type, blocked, start, goal, t_now, t_first);
    const auto t1 = std::chrono::steady_clock::now();
    const double dt =
        std::chrono::duration<double>(t1 - t0).count();

    BenchRow row;
    row.target_vertices = target_v;
    row.actual_nodes = grid_n * grid_n;
    row.grid_n = grid_n;
    row.n_types = n_types;
    row.decision_time_s = dt;
    row.W_star = dec.W_star;
    rows.push_back(row);
  }

  // Summary for this cell
  double sum = 0.0;
  for (const auto& r : rows) sum += r.decision_time_s;
  std::vector<double> times;
  times.reserve(rows.size());
  for (const auto& r : rows) times.push_back(r.decision_time_s);
  std::sort(times.begin(), times.end());
  const double mean = sum / rows.size();
  const double p95 = times[static_cast<size_t>(0.95 * (times.size() - 1))];
  std::cout << "  v" << target_v << "_k" << n_types
            << ": mean=" << mean << "s p95=" << p95 << "s"
            << " W*=" << rows.front().W_star << "\n";
  return rows;
}

void writeCsv(const std::string& path, const std::vector<BenchRow>& rows) {
  std::ofstream out(path);
  out << "backend,target_vertices,actual_nodes,grid_n,n_types,decision_time_s,W_star\n";
  out << std::setprecision(10);
  for (const auto& r : rows) {
    out << "cpp_microbench," << r.target_vertices << "," << r.actual_nodes << ","
        << r.grid_n << "," << r.n_types << "," << r.decision_time_s << ","
        << r.W_star << "\n";
  }
}

void printUsage(const char* argv0) {
  std::cerr
      << "Usage: " << argv0
      << " [--out PATH] [--trials N] [--only v50_k4]\n"
      << "  Sweeps scalability (V,K) matching Python configs/scalability_*.yaml\n"
      << "  Graphs: exact Python _create_grid_graph (L=0.3m, speed=0.95m/s)\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string out_path = "scalability_cpp_timings.csv";
  int trials = kDefaultTrials;
  std::string only;  // e.g. v50_k4

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--out" && i + 1 < argc) {
      out_path = argv[++i];
    } else if (a == "--trials" && i + 1 < argc) {
      trials = std::stoi(argv[++i]);
    } else if (a == "--only" && i + 1 < argc) {
      only = argv[++i];
    } else if (a == "-h" || a == "--help") {
      printUsage(argv[0]);
      return 0;
    } else {
      printUsage(argv[0]);
      return 1;
    }
  }

  // Quiet logs so CLOG(INFO) does not dominate wall-clock.
  vtr::logging::configureLogging("", false);
  el::Configurations conf;
  conf.setToDefault();
  conf.setGlobally(el::ConfigurationType::Enabled, "false");
  el::Loggers::reconfigureAllLoggers(conf);

  std::mt19937 rng(kRngSeed);
  std::vector<BenchRow> all_rows;

  std::cout << "C++ W* scalability microbench\n"
            << "  L=" << kSegmentLengthM << "m speed=" << kSpeedMps
            << " W/T grids=" << kWGridPoints << " trials=" << trials << "\n";

  // Parity check all grid sizes first
  std::cout << "Graph parity checks:\n";
  for (int v : kTargetVertices) {
    const int n = kGridNForTarget.at(v);
    auto g = buildPythonParityGrid(n, kSegmentLengthM, kSpeedMps);
    assertGridInvariants(g);
  }

  for (int v : kTargetVertices) {
    for (int k : kNumTypesLevels) {
      std::ostringstream tag;
      tag << "v" << v << "_k" << k;
      if (!only.empty() && only != tag.str()) continue;
      std::cout << "Running " << tag.str() << "...\n";
      auto rows = runCell(v, k, trials, rng);
      all_rows.insert(all_rows.end(), rows.begin(), rows.end());
    }
  }

  if (all_rows.empty()) {
    std::cerr << "No cells run (check --only).\n";
    return 1;
  }

  writeCsv(out_path, all_rows);
  std::cout << "Wrote " << all_rows.size() << " rows to " << out_path << "\n";
  return 0;
}
