#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_navigation/assemblies/base_assembly.hpp>

namespace vtr {
namespace navigation {

void BaseAssembly::run(QueryCache &qdata, MapCache &mdata,
                       const std::shared_ptr<const Graph> &graph) {
#if false
  using Timer = common::timing::SimpleTimer;
  Timer total_timer;
  std::map<std::string, std::map<std::string, double>> time_map;
  for (auto m : modules_) {
    Timer module_timer;
    auto module_name = m ? typeid(*m).name() : "(nullptr)";
    try {
      m->runWrapper(qdata, mdata, graph);
    } catch (std::exception &e) {
      LOG(WARNING) << "assembly: <" << typeid(*this).name() << "> module: <"
                   << module_name << "::run> threw an unhandled exception "
                   << e.what();
    } catch (...) {
      LOG(WARNING) << "assembly: <" << typeid(*this).name() << "> module: <"
                   << module_name << "::run> threw an unhandled exception.";
      throw;
    }
    time_map[module_name]["run"] = module_timer.elapsedMs();
  }
  for (auto m : modules_) {
    Timer module_timer;
    auto module_name = m ? typeid(*m).name() : "(nullptr)";
    try {
      m->visualize(qdata, mdata, graph);
    } catch (std::exception &e) {
      LOG(WARNING) << "assembly: <" << typeid(*this).name() << "> module: <"
                   << module_name
                   << "::visualize> threw an unhandled exception " << e.what();
    } catch (...) {
      auto name = m ? typeid(*m).name() : "(nullptr)";
      LOG(WARNING) << "assembly: <" << typeid(*this).name() << "> module: <"
                   << name << "::visualize> threw an unhandled exception.";
      throw;
    }
    time_map[module_name]["visualize"] = module_timer.elapsedMs();
  }

  double total_time = total_timer.elapsedMs();
  if (total_time >= 300) {
    std::stringstream timing_ss;
    for (const auto m : time_map) {
      timing_ss << m.first << " ";
      for (const auto &f : m.second) {
        timing_ss << ":: " << f.first << ": " << f.second << " ";
      }
      timing_ss << "\n";
    }
    auto timing_str = timing_ss.str();
    LOG(WARNING) << "Assembly: " << typeid(*this).name() << " took "
                 << total_time << " ms:\n"
                 << timing_str.substr(0, timing_str.size() - 1);
  }
#endif
}

void BaseAssembly::updateGraph(QueryCache &qdata, MapCache &mdata,
                               const std::shared_ptr<Graph> &graph,
                               const VertexId &live_id) {
#if false
  using Timer = common::timing::SimpleTimer;
  Timer total_timer;
  std::map<std::string, std::map<std::string, double>> time_map;
  for (auto m : modules_) {
    Timer module_timer;
    auto module_name = m ? typeid(*m).name() : "(nullptr)";
    try {
      m->updateGraphWrapper(qdata, mdata, graph, live_id);
    } catch (std::exception &e) {
      LOG(WARNING) << "assembly: <" << typeid(*this).name() << "> module: <"
                   << module_name
                   << "::updateGraph> threw an unhandled exception "
                   << e.what();
    } catch (...) {
      auto name = m ? typeid(*m).name() : "(nullptr)";
      LOG(WARNING) << "assembly: <" << typeid(*this).name() << "> module: <"
                   << name << "::updateGraph> threw an unhandled exception.";
      throw;
    }
    time_map[module_name]["update"] = module_timer.elapsedMs();
  }

  double total_time = total_timer.elapsedMs();
  if (total_time > 300) {
    std::stringstream timing_ss;
    for (const auto m : time_map) {
      timing_ss << m.first << " ";
      for (const auto &f : m.second) {
        timing_ss << ":: " << f.first << ": " << f.second << " ";
      }
      timing_ss << "\n";
    }
    auto timing_str = timing_ss.str();
    LOG(WARNING) << "Assembly: " << typeid(*this).name() << " took "
                 << total_time << " ms:\n"
                 << timing_str.substr(0, timing_str.size() - 1);
  }
#endif
}

}  // namespace navigation
}  // namespace vtr
