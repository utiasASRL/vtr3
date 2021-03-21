#include <vtr_common/utils/filesystem.hpp>
#include <vtr_navigation/assemblies.hpp>  // TODO: should be created here?
#include <vtr_navigation/factories/ros_assembly_factory.hpp>
#include <vtr_navigation/factories/ros_tactic_factory.hpp>
#include <vtr_navigation/tactics/tactic_config.hpp>
#if false
#include <vtr_logging/logging.hpp>     // for debugging only
#include <vtr_navigation/modules.hpp>  // TODO: should be created here?
#include <vtr_navigation/tactics.hpp>
#include <vtr_path_tracker/base.hpp>  // RCGraph
#endif
 #include <vtr_navigation/factories/path_tracker_factory.hpp>
namespace vtr {
namespace navigation {

ROSTacticFactory::tac_ptr ROSTacticFactory::make_str(
    const std::string& type_str) const {
  /// \todo the following block of code looks a bit redundant, as one needs to
  /// create all assemblies in make_str even some of them may not be needed. I
  /// think we need a better way of creating these assemblies from tactic class.
  /// Get parameters from ROS
  ROSAssemblyFactory converter_builder{node_, "converter"};
  ROSAssemblyFactory qvo_builder{node_, "quick_vo"};
  ROSAssemblyFactory rvo_builder{node_, "refined_vo"};
  ROSAssemblyFactory loc_builder{node_, "loc"};

  std::shared_ptr<ConverterAssembly> converter;
  std::shared_ptr<QuickVoAssembly> quick_vo;
  std::shared_ptr<RefinedVoAssembly> refined_vo;
  std::shared_ptr<LocalizerAssembly> localizer;
  std::shared_ptr<vtr::path_tracker::Base> path_tracker;

  // Build the assemblies from the parameters
  try {
    converter = std::dynamic_pointer_cast<ConverterAssembly>(
        converter_builder.makeVerified());
    if (!converter)
      throw std::runtime_error("nullptr returned after converter cast");

    quick_vo =
        std::dynamic_pointer_cast<QuickVoAssembly>(qvo_builder.makeVerified());
    if (!quick_vo) throw std::runtime_error("nullptr returned after qvo cast");

    refined_vo = std::dynamic_pointer_cast<RefinedVoAssembly>(
        rvo_builder.makeVerified());
    if (!refined_vo)
      throw std::runtime_error("nullptr returned after rvo cast");

    localizer = std::dynamic_pointer_cast<LocalizerAssembly>(
        loc_builder.makeVerified());
    if (!localizer) throw std::runtime_error("nullptr returned after loc cast");

  } catch (std::exception& e) {
    LOG(ERROR) << "exception while building assemblies: " << e.what();
    throw e;
  }
  // Set up tactic configuration
  TacticConfig config;
  // clang-format off
  config.robot_index = node_->declare_parameter<decltype(config.robot_index)>(param_prefix_ + ".robot_index", -1);
  config.extrapolate_VO = node_->declare_parameter<decltype(config.extrapolate_VO)>(param_prefix_ + ".extrapolate_VO", true);
  config.extrapolate_timeout = node_->declare_parameter<decltype(config.extrapolate_timeout)>(param_prefix_ + ".extrapolate_timeout", 1.0);
  config.insert_initial_run = node_->declare_parameter<decltype(config.insert_initial_run)>(param_prefix_ + ".insert_initial_run", false);
  config.keyframe_parallelization = node_->declare_parameter<decltype(config.keyframe_parallelization)>(param_prefix_ + ".keyframe_parallelization", true);
  config.keyframe_skippable = node_->declare_parameter<decltype(config.keyframe_skippable)>(param_prefix_ + ".keyframe_skippable", true);
  config.localization_parallelization = node_->declare_parameter<decltype(config.localization_parallelization)>(param_prefix_ + ".localization_parallelization", true);
  config.localization_skippable = node_->declare_parameter<decltype(config.localization_skippable)>(param_prefix_ + ".localization_skippable", true);

  auto ldt = node_->declare_parameter<std::vector<double>>(param_prefix_ + ".loc_deadreckoning_thresh", std::vector<double>{});
  if (ldt.size() != 3) {
    LOG(WARNING)
        << "Localization deadreckoning covariance threshold malformed ("
        << ldt.size() << "elements). Must be 3 elements!";
  }
  // make at least size elements to prevent segfault
  if (ldt.size() < 3) ldt.resize(3, 0);
  config.loc_deadreckoning_thresh << ldt[0], ldt[1], ldt[2];

  auto llt = node_->declare_parameter<std::vector<double>>(param_prefix_ + ".loc_lost_thresh", std::vector<double>{});
  if (llt.size() != 3) {
    LOG(WARNING) << "Localization lost covariance threshold malformed ("
                 << llt.size() << "elements). Must be 3 elements!";
  }
  // make at least size elements to prevent segfault
  if (llt.size() < 3) llt.resize(3, 0);
  config.loc_lost_thresh << llt[0], llt[1], llt[2];

  // Get the number of parallel processing threads for the parallel tactic from
  // ROS param
  int num_threads = node_->declare_parameter<int>(param_prefix_ + ".num_threads", 4);
  /// \todo Shouldn't we throw an error instead of silently change num_threads
  /// tos >=1?
  num_threads = std::max(1, num_threads);

#if false
  // set up memory config
  config.map_memory_config.enable = node_->declare_parameter<decltype(config.map_memory_config.enable)>("map_memory.enable", true);
  config.map_memory_config.vertex_life_span = node_->declare_parameter<decltype(config.map_memory_config.vertex_life_span)>("map_memory.vertex_life_span", 50);
  config.map_memory_config.lookahead_distance = node_->declare_parameter<decltype(config.map_memory_config.lookahead_distance)>("map_memory.lookahead_distance", 100);
  config.map_memory_config.streams_to_load = node_->declare_parameter<decltype(config.map_memory_config.streams_to_load)>("map_memory.streams_to_load", true);
  config.map_memory_config.priv_streams_to_load = node_->declare_parameter<decltype(config.map_memory_config.priv_streams_to_load)>("map_memory.priv_streams_to_load", true);
  config.live_memory_config.lookahead_distance = config.map_memory_config.lookahead_distance;
  config.live_memory_config.enable = node_->declare_parameter<decltype(config.live_memory_config.enable)>("live_memory.enable", true);
  config.live_memory_config.window_size = node_->declare_parameter<decltype(config.live_memory_config.window_size)>("live_memory.window_size", 250);
#endif
  // set up localization chain
  config.locchain_config.min_cusp_distance = node_->declare_parameter<decltype(config.locchain_config.min_cusp_distance)>("locchain.min_cusp_distance", 1.5);
  config.locchain_config.angle_weight = node_->declare_parameter<decltype(config.locchain_config.angle_weight)>("locchain.angle_weight", 7.0);
  config.locchain_config.search_depth = node_->declare_parameter<decltype(config.locchain_config.search_depth)>("locchain.search_depth", 20);
  config.locchain_config.search_back_depth = node_->declare_parameter<decltype(config.locchain_config.search_back_depth)>("locchain.search_back_depth",10);
  config.locchain_config.distance_warning = node_->declare_parameter<decltype(config.locchain_config.distance_warning)>("locchain.distance_warning", 3);

  // clang-format on
  // set up pipeline config
  auto dlc = node_->declare_parameter<std::vector<double>>(
      "mergepipeline.default_loc_cov", std::vector<double>{});
  if (dlc.size() != 6) {
    LOG(WARNING) << "Merge pipeline default localization covariance malformed ("
                 << dlc.size() << " elements). Must be 6 elements!";
  }
  // make at least size elements to prevent segfault
  if (dlc.size() < 6) dlc.resize(6, 1.);
  config.pipeline_config.default_loc_cov.setZero();
  config.pipeline_config.default_loc_cov.diagonal() << dlc[0], dlc[1], dlc[2],
      dlc[3], dlc[4], dlc[5];

  config.pipeline_config.loc_search_step =
      node_
          ->declare_parameter<decltype(config.pipeline_config.loc_search_step)>(
              "searchpipeline.search_step", 0);
  // get the boolean that defines whether we should integrate poses to get a
  // localization prior
  config.pipeline_config.use_integrated_loc_prior =
      node_->declare_parameter<decltype(
          config.pipeline_config.use_integrated_loc_prior)>(
          "mergepipeline.use_integrated_loc_prior", false);

  // misc config
  auto data_dir =
      node_->has_parameter("data_dir")
          ? node_->get_parameter("data_dir").get_value<std::string>()
          : node_->declare_parameter<decltype(config.data_directory)>(
                "data_dir", "");
  config.data_directory = common::utils::expand_user(data_dir);
  if (config.data_directory == "") {
    // we cannot do anything without a data directory.
    LOG(WARNING) << "Initialized Navigator without directory; set it manually "
                    "before continuing.";
    return nullptr;
  }

  // graph config
  auto use_existing_graph = node_->declare_parameter<bool>(
      param_prefix_ + ".use_existing_graph", true);
  config.graph_index =
      node_->declare_parameter<decltype(config.graph_index)>("graph_index", 0);

  std::shared_ptr<pose_graph::RCGraph> graph;
  if (use_existing_graph) {
    LOG(INFO) << "Loading graph " << std::to_string(config.graph_index)
              << ".....";
    // Load an existing graph, or create a new one
    graph = pose_graph::RCGraph::LoadOrCreate(
        config.data_directory + "/graph" + std::to_string(config.graph_index) +
            ".index",
        config.graph_index);
    LOG(INFO) << "Loaded graph has " << graph->vertices()->size()
              << " vertices";
  } else {
    LOG(INFO) << "Creating new graph at " << config.data_directory;
    graph.reset(new pose_graph::RCGraph(config.data_directory + "/graph" +
                                            std::to_string(config.graph_index) +
                                            ".index",
                                        config.graph_index));
  }

  LOG(INFO) << "Initializing tactic.....";
  // Make a parallel or basic tactic
  ROSTacticFactory::tac_ptr tactic;
  if (!type_str.compare(std::string("basic"))) {
    tactic.reset(new navigation::BasicTactic(config, converter, quick_vo,
                                             refined_vo, localizer, graph));
  }
#if false
  else if (!type_str.compare(std::string("parallel"))) {
    tactic.reset(new navigation::ParallelTactic(config, uint32_t(num_threads),
                                                converter, quick_vo, refined_vo,
                                                localizer, graph));
  }
#endif
  else {
    LOG(ERROR) << "Couldn't determine tactic type: \"" << type_str << "\"";
    return nullptr;
  }

  // build the path tracker
  // TODO (old): configs come in here...
  std::string pt_type;
  pt_type = node_->declare_parameter<std::string>(param_prefix_ + ".path_tracker_type", "robust_mpc_path_tracker");

  vtr::path_tracker::PathTrackerFactory pt_factory(graph, node_);
    try {
      path_tracker = pt_factory.create(pt_type);
      if (!path_tracker)
        throw std::runtime_error("nullptr returned after path tracker build");
    } catch (std::exception& e) {
      LOG(ERROR) << "exception while building path tracker: " << e.what();
      throw e;
    }

    tactic->setPathTracker(path_tracker);

  return tactic;
}

}  // namespace navigation
}  // namespace vtr
