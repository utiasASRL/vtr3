// Standalone smoke test for the SPARROW POMCP planner core.
// Graph:  0 --10s-- 1 --10s-- 3(goal)   direct route
//         0 --Ds--  2 --Ds--  3         detour route (D varies per scenario)
// Edge (0,1) is blocked by an obstacle with mean duration 60 s.
//   Scenario A: D = 100  -> detour costs 200 s, waiting ~80 s  => expect MAXWAIT
//   Scenario B: D = 15   -> detour costs 30 s,  waiting ~80 s  => expect TRAVERSE via 2
#include <cassert>
#include <iostream>

#include "vtr_logging/logging_init.hpp"  // defines easylogging storage
#include "vtr_navigation/sparrow_planner.hpp"

using namespace vtr::navigation::sparrow;

static PlanResult run_scenario(double detour_leg_s) {
  std::map<SVertex, std::vector<SVertex>> nbrs = {
      {0, {1, 2}}, {1, {0, 3}}, {2, {0, 3}}, {3, {1, 2}}};
  std::map<SEdge, double> tt = {
      {canonical_edge(0, 1), 10.0}, {canonical_edge(1, 3), 10.0},
      {canonical_edge(0, 2), detour_leg_s}, {canonical_edge(2, 3), detour_leg_s}};
  GraphContext ctx(nbrs, tt, /*goal=*/3);

  SparrowModel model;
  model.class_names = {"pedestrian"};
  model.class_probs = {1.0};
  model.p_block = 0.05;
  model.num_edges = static_cast<int>(ctx.edges.size());
  model.prior_residual_mean = 60.0;
  model.spawn_rate_hz = model.p_block * model.num_edges / model.prior_residual_mean;
  model.km = nullptr;  // exponential fallback

  LocalObservation local;
  local.vertex = 0;
  local.time = 0.0;
  local.statuses[canonical_edge(0, 1)] = 1;  // blocked
  local.statuses[canonical_edge(0, 2)] = 0;  // free
  local.ages[canonical_edge(0, 1)] = 5.0;
  local.labels[canonical_edge(0, 1)] = "pedestrian";

  ParticleBelief belief(&ctx, &model, /*num_particles=*/300,
                        /*planner_no_adjacent_blocking=*/false, /*seed=*/42);
  belief.initialize(local);

  TransitionModel tm(&ctx, /*delta_obs=*/3.0,
                     /*wait_set=*/{5.0, 10.0, 20.0, 30.0, 60.0},
                     /*allow_observe=*/true, /*corridor_traversal=*/true,
                     /*duration_bin_width=*/1.0);

  std::vector<SAction> roots;
  SAction w; w.kind = SAction::MAXWAIT; w.edge = canonical_edge(0, 1); w.W = 30.0;
  roots.push_back(w);
  SAction t; t.kind = SAction::TRAVERSE; t.edge = canonical_edge(0, 2); t.first_hop = 2;
  roots.push_back(t);

  SparrowSearchSettings st;
  st.num_simulations = 3000;
  st.max_planning_time_s = 10.0;
  st.max_depth = 40;
  st.max_sim_time_s = 600.0;
  st.c_uct = 30.0;

  SparrowSolver solver(&tm, st, /*seed=*/7);
  return solver.plan(belief.particles(), roots);
}

static void report(const char* name, const PlanResult& r) {
  std::cout << name << ": sims=" << r.simulations
            << " time=" << r.planning_time_s << "s nodes=" << r.tree_nodes
            << " depth=" << r.tree_depth << "\n";
  for (const auto& s : r.root_actions)
    std::cout << "  " << s.action.str() << "  visits=" << s.visits
              << "  Q=" << s.q_cost << "\n";
  std::cout << "  chosen: " << (r.action ? r.action->str() : "<none>") << "\n";
}

int main() {
  auto a = run_scenario(100.0);
  report("Scenario A (expensive detour, expect MAXWAIT)", a);
  assert(a.action && a.action->kind == SAction::MAXWAIT);

  auto b = run_scenario(15.0);
  report("Scenario B (cheap detour, expect TRAVERSE via 2)", b);
  assert(b.action && b.action->kind == SAction::TRAVERSE &&
         b.action->first_hop == 2);

  std::cout << "SMOKE TEST PASSED\n";
  return 0;
}
