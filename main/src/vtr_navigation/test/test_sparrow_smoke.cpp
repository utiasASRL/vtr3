// Standalone smoke test for the SPARROW POMCP planner core.
// Graph:  0 --10s-- 1 --10s-- 3(goal)   direct route
//         0 --Ds--  2 --Ds--  3         detour route (D varies per scenario)
// Edge (0,1) is blocked by an obstacle with mean duration 60 s.
//   Scenario A: D = 100  -> detour costs 200 s, waiting ~80 s  => expect MAXWAIT
//   Scenario B: D = 15   -> detour costs 30 s,  waiting ~80 s  => expect TRAVERSE via 2
// Scenario C: front obstacle UNLABELED, classes "pedestrian" (fast clear) and
//             "car" (very slow) both KM-fitted -> Observe (cost 3 s) resolves
//             which world we are in and should beat blind waiting/detouring.
#include <cassert>
#include <iostream>

#include "vtr_logging/logging_init.hpp"  // defines easylogging storage
#include "vtr_navigation/sparrow_planner.hpp"
#include "vtr_navigation/survival_model.hpp"

using namespace vtr::navigation::sparrow;
using vtr::navigation::SurvivalModel;

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

// Scenario C: the value of information. The front edge (0,1) is blocked by an
// UNLABELED obstacle. pedestrian clears in ~15 s, car in ~600 s (50/50 prior,
// both KM-fitted). Detour costs 120 s. Knowing the class flips the decision
// (pedestrian -> wait ~35 s total, car -> detour 120 s), so paying 3 s to
// Observe should beat committing blind either way.
static PlanResult run_observe_scenario() {
  std::map<SVertex, std::vector<SVertex>> nbrs = {
      {0, {1, 2}}, {1, {0, 3}}, {2, {0, 3}}, {3, {1, 2}}};
  std::map<SEdge, double> tt = {
      {canonical_edge(0, 1), 10.0}, {canonical_edge(1, 3), 10.0},
      {canonical_edge(0, 2), 60.0}, {canonical_edge(2, 3), 60.0}};
  GraphContext ctx(nbrs, tt, /*goal=*/3);

  static SurvivalModel sm;  // static: outlives the frozen model snapshot
  for (int i = 0; i < 30; ++i) {
    sm.addSample("pedestrian", 12.0 + (i % 7), false, i);
    sm.addSample("car", 550.0 + 10.0 * (i % 10), false, i);
  }

  SparrowModel model;
  model.class_names = {"pedestrian", "car"};
  model.class_probs = {0.5, 0.5};
  model.p_block = 0.05;
  model.num_edges = static_cast<int>(ctx.edges.size());
  model.prior_residual_mean = 60.0;
  model.km = &sm;
  model.fitted_classes = {"pedestrian", "car"};
  model.spawn_rate_hz = model.p_block * model.num_edges / 300.0;

  LocalObservation local;
  local.vertex = 0;
  local.time = 0.0;
  local.statuses[canonical_edge(0, 1)] = 1;  // blocked, NO label
  local.statuses[canonical_edge(0, 2)] = 0;  // free
  local.ages[canonical_edge(0, 1)] = 2.0;

  ParticleBelief belief(&ctx, &model, /*num_particles=*/400,
                        /*planner_no_adjacent_blocking=*/false, /*seed=*/11);
  belief.initialize(local);

  TransitionModel tm(&ctx, /*delta_obs=*/3.0, {5.0, 10.0, 20.0, 30.0},
                     /*allow_observe=*/true, /*corridor_traversal=*/true,
                     /*duration_bin_width=*/1.0);

  std::vector<SAction> roots;
  SAction w; w.kind = SAction::MAXWAIT; w.edge = canonical_edge(0, 1); w.W = 20.0;
  roots.push_back(w);
  SAction t; t.kind = SAction::TRAVERSE; t.edge = canonical_edge(0, 2); t.first_hop = 2;
  roots.push_back(t);
  SAction o; o.kind = SAction::OBSERVE; o.edge = canonical_edge(0, 1);
  roots.push_back(o);

  SparrowSearchSettings st;
  st.num_simulations = 8000;
  st.max_planning_time_s = 20.0;
  st.max_depth = 40;
  st.max_sim_time_s = 1200.0;
  st.c_uct = 30.0;

  SparrowSolver solver(&tm, st, /*seed=*/13);
  return solver.plan(belief.particles(), roots);
}

int main() {
  auto a = run_scenario(100.0);
  report("Scenario A (expensive detour, expect MAXWAIT)", a);
  assert(a.action && a.action->kind == SAction::MAXWAIT);

  auto b = run_scenario(15.0);
  report("Scenario B (cheap detour, expect TRAVERSE via 2)", b);
  assert(b.action && b.action->kind == SAction::TRAVERSE &&
         b.action->first_hop == 2);

  auto c = run_observe_scenario();
  report("Scenario C (unlabeled, class matters, expect OBSERVE)", c);
  assert(c.action && c.action->kind == SAction::OBSERVE);

  std::cout << "SMOKE TEST PASSED\n";
  return 0;
}
