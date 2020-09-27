#include <vtr_navigation/assemblies.hpp>  // TODO: should be created here?
#include <vtr_navigation/factories/ros_assembly_factory.hpp>
#include <vtr_navigation/factories/ros_tactic_factory.hpp>
#if false
#include <vtr_navigation/modules.hpp>  // TODO: should be created here?
#include <vtr_navigation/tactics.hpp>
#include <vtr_navigation/tactics/tactic_config.hpp>

#include <vtr_common/logging.hpp>  // for debugging only

#include <vtr_path_tracker/base.hpp>  // RCGraph
// #include <vtr_path_tracker/path_tracker_factory.hpp>
#endif
namespace vtr {
namespace navigation {

ROSTacticFactory::tac_ptr ROSTacticFactory::make_str(
    const std::string& type_str) const {
  /// \todo the following block of code looks a bit redundant, as one needs to
  /// create all assemblies in make_str even some of them may not be needed. I
  /// think we need a better way of creating these assemblies from tactic class.
  /// Get parameters from ROS
  ROSAssemblyFactory converter_builder{node_, "converter"};
#if false
  ROSAssemblyFactory qvo_builder{node_, "quick_vo"};
  ROSAssemblyFactory rvo_builder{node_, "refined_vo"};
  ROSAssemblyFactory loc_builder{node_, "loc"};
#endif

  std::shared_ptr<ConverterAssembly> converter;
#if false
  std::shared_ptr<QuickVoAssembly> quick_vo;
  std::shared_ptr<RefinedVoAssembly> refined_vo;
  std::shared_ptr<LocalizerAssembly> localizer;
#if 0
  std::shared_ptr<asrl::path_tracker::Base> path_tracker;
#endif
#endif
  // Build the assemblies from the parameters
  try {
    converter = std::dynamic_pointer_cast<navigation::ConverterAssembly>(
        converter_builder.makeVerified());
    if (!converter)
      throw std::runtime_error("nullptr returned after converter cast");
#if false
    quick_vo = std::dynamic_pointer_cast<navigation::QuickVoAssembly>(
        qvo_builder.makeVerified());
    if (!quick_vo) throw std::runtime_error("nullptr returned after qvo cast");

    refined_vo = std::dynamic_pointer_cast<navigation::RefinedVoAssembly>(
        rvo_builder.makeVerified());
    if (!refined_vo)
      throw std::runtime_error("nullptr returned after rvo cast");
    localizer = std::dynamic_pointer_cast<navigation::LocalizerAssembly>(
        loc_builder.makeVerified());
    if (!localizer) throw std::runtime_error("nullptr returned after loc cast");
#endif
  } catch (std::exception& e) {
    LOG(ERROR) << "exception while building assemblies: " << e.what();
    throw e;
  }
#if false
  // Set up tactic configuration
  TacticConfig config;
  nh_->param<int>(param_prefix_ + "robot_index", config.robot_index, -1);
  nh_->param<bool>(param_prefix_ + "extrapolate_VO", config.extrapolate_VO,
                   true);
  nh_->param<double>(param_prefix_ + "extrapolate_timeout",
                     config.extrapolate_timeout, 1.0);
  nh_->param<bool>(param_prefix_ + "insert_initial_run",
                   config.insert_initial_run, false);
  nh_->param<bool>(param_prefix_ + "keyframe_parallelization",
                   config.keyframe_parallelization, true);
  nh_->param<bool>(param_prefix_ + "keyframe_skippable",
                   config.keyframe_skippable, true);
  nh_->param<bool>(param_prefix_ + "localization_parallelization",
                   config.localization_parallelization, true);
  nh_->param<bool>(param_prefix_ + "localization_skippable",
                   config.localization_skippable, true);
#if 0
  nh_->param<bool>(param_prefix_ + "ta_parallelization",
                   config.ta_parallelization, true);
#endif
  std::vector<double> ldt;  // loc deadreckoning thresh
  nh_->getParam(param_prefix_ + "loc_deadreckoning_thresh", ldt);
  if (ldt.size() != 3) {
    LOG(WARNING)
        << "Localization deadreckoning covariance threshold malformed ("
        << ldt.size() << "elements). Must be 3 elements!";
  }
  // make at least size elements to prevent segfault
  if (ldt.size() < 3) {
    ldt.resize(3, 0);
  }
  config.loc_deadreckoning_thresh << ldt[0], ldt[1], ldt[2];

  std::vector<double> llt;  // loc lost thresh
  nh_->getParam(param_prefix_ + "loc_lost_thresh", llt);
  if (llt.size() != 3) {
    LOG(WARNING) << "Localization lost covariance threshold malformed ("
                 << llt.size() << "elements). Must be 3 elements!";
  }
  // make at least size elements to prevent segfault
  if (llt.size() < 3) {
    llt.resize(3, 0);
  }
  config.loc_lost_thresh << llt[0], llt[1], llt[2];

  // Get the number of parallel processing threads for the parallel tactic from
  // ROS param
  int num_threads;
  nh_->param<int>(param_prefix_ + "num_threads", num_threads, 4);
  /// \todo Shouldn't we throw an error instead of silently change num_threads
  /// tos >=1?
  num_threads = std::max(1, num_threads);

  // set up memory config
  nh_->param<bool>("map_memory/enable", config.map_memory_config.enable, true);
  nh_->param<int>("map_memory/vertex_life_span",
                  config.map_memory_config.vertex_life_span, 50);
  nh_->param<int>("map_memory/lookahead_distance",
                  config.map_memory_config.lookahead_distance, 100);
  nh_->param<std::vector<std::string>>("map_memory/streams_to_load",
                                       config.map_memory_config.streams_to_load,
                                       std::vector<std::string>());
  nh_->param<std::vector<std::string>>(
      "map_memory/priv_streams_to_load",
      config.map_memory_config.priv_streams_to_load,
      std::vector<std::string>());

  config.live_memory_config.lookahead_distance =
      config.map_memory_config.lookahead_distance;  // copy across
  nh_->param<bool>("live_memory/enable", config.live_memory_config.enable,
                   true);
  nh_->param<int>("live_memory/window_size",
                  config.live_memory_config.window_size, 250);

  // set up localization chain
  nh_->param<double>("locchain/min_cusp_distance",
                     config.locchain_config.min_cusp_distance, 1.5);
  nh_->param<double>("locchain/angle_weight",
                     config.locchain_config.angle_weight, 7.0);
  nh_->param<int>("locchain/search_depth", config.locchain_config.search_depth,
                  20);
  nh_->param<int>("locchain/search_back_depth",
                  config.locchain_config.search_back_depth, 10);
  nh_->param<double>("locchain/distance_warning",
                     config.locchain_config.distance_warning, 3);

  // set up pipeline config
  std::vector<double> dlc;  // default localisation covariance
  nh_->getParam("mergepipeline/default_loc_cov", dlc);
  if (dlc.size() != 6) {
    LOG(WARNING) << "Merge pipeline default localization covariance malformed ("
                 << dlc.size() << "elements). Must be 6 elements!";
  }
  // make at least size elements to prevent segfault
  if (dlc.size() < 6) {
    dlc.resize(6, 1.);
  }
  config.pipeline_config.default_loc_cov.setZero();
  config.pipeline_config.default_loc_cov.diagonal() << dlc[0], dlc[1], dlc[2],
      dlc[3], dlc[4], dlc[5];

  nh_->getParam("searchpipeline/search_step",
                config.pipeline_config.loc_search_step);

  // get the boolean that defines whether we should integrate poses to get a
  // localization prior
  nh_->param<bool>("mergepipeline/use_integrated_loc_prior",
                   config.pipeline_config.use_integrated_loc_prior, false);

  // misc config
  nh_->param<std::string>("data_dir", config.data_directory, "");
  if (config.data_directory ==
      "") {  // We cannot do anything without a data directory
    LOG(WARNING) << "Initialized Navigator without directory; set it manually "
                    "before continuing.";
    return nullptr;
  }
  // graph config
  bool use_existing_graph = true;
  nh_->param<bool>(param_prefix_ + "use_existing_graph", use_existing_graph,
                   true);
  nh_->param<int>("graph_index", config.graph_index, 0);

  std::shared_ptr<asrl::pose_graph::RCGraph> graph;
  if (use_existing_graph) {
    LOG(INFO) << "Loading graph " << std::to_string(config.graph_index)
              << ".....";
    // Load an existing graph, or create a new one
    graph = asrl::pose_graph::RCGraph::LoadOrCreate(
        config.data_directory + "/graph" + std::to_string(config.graph_index) +
            ".index",
        config.graph_index);
    LOG(INFO) << "Loaded graph has " << graph->vertices()->size()
              << " vertices";
  } else {
    LOG(INFO) << "Creating new graph at " << config.data_directory;
    graph.reset(new asrl::pose_graph::RCGraph(
        config.data_directory + "/graph" + std::to_string(config.graph_index) +
            ".index",
        config.graph_index));
  }

  LOG(INFO) << "Initializing tactic.....";
#endif
  // Make a parallel or basic tactic
  ROSTacticFactory::tac_ptr tactic;
#if false
  if (!type_str.compare(std::string("basic"))) {
    tactic.reset(new navigation::BasicTactic(config, converter, quick_vo,
                                             refined_vo, localizer,
                                             /*terrain_assessment,*/
                                             graph));
  } else if (!type_str.compare(std::string("parallel"))) {
    tactic.reset(new navigation::ParallelTactic(config, uint32_t(num_threads),
                                                converter, quick_vo, refined_vo,
                                                localizer,
                                                /*terrain_assessment,*/
                                                graph));
  } else {
    LOG(ERROR) << "Couldn't determine tactic type: \"" << type_str << "\"";
    return nullptr;
  }

#if 0
  // build the path tracker
  // TODO: configs come in here...
  bool use_new_pt;
  nh_->param<bool>("/using_new_path_tracker", use_new_pt, false);
  if (use_new_pt) {
    std::string pt_type;
    nh_->param<std::string>("/path_tracker_type", pt_type,
                            "robust_mpc_path_tracker");
    asrl::path_tracker::PathTrackerFactory pt_factory(graph, nh_);
    try {
      path_tracker = pt_factory.create(pt_type);
      if (!path_tracker)
        throw std::runtime_error("nullptr returned after path tracker build");
    } catch (std::exception& e) {
      LOG(ERROR) << "exception while building path tracker: " << e.what();
      throw e;
    }

    tactic->setPathTracker(path_tracker);
  }
#endif

#endif
  return tactic;
}

}  // namespace navigation
}  // namespace vtr
