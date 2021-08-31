#include <filesystem>
// #include <c++/8/fstream>
#include <fstream>
#include <iostream>
#include <random>

#include <vtr_logging/logging_init.hpp>
#include <vtr_messages/msg/rig_landmarks.hpp>
#include <vtr_messages/msg/localization_status.hpp>
#include <vtr_messages/msg/exp_recog_status.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#include <vtr_pose_graph/path/path.hpp>

namespace fs = std::filesystem;
using namespace vtr::pose_graph;

void ReadLocalizationResults(std::string graph_dir, std::string results_dir,
                             int* num_fail_loc_all, int* num_fail_exp_all,
                             int* num_fail_read_exp_all) {

  // Load the graph
  std::shared_ptr<RCGraph> graph;
  graph = vtr::pose_graph::RCGraph::LoadOrCreate(graph_dir, 0);
  LOG(INFO) << "Loaded graph has " << graph->vertices()->size() << " vertices";

  // Register the stream so we can read messages from it
  std::string stream_name_loc = "results_localization";
  std::string stream_name_exp = "experience_triage";
  int r_ind = 0;
  for (const auto& run : graph->runs()) {
    // We are iterating over the teach and one repeat
    if (r_ind > 0) { 
      run.second->registerVertexStream<vtr_messages::msg::LocalizationStatus>(
        stream_name_loc, true, RegisterMode::Existing);
      run.second->registerVertexStream<vtr_messages::msg::ExpRecogStatus>(
        stream_name_exp, true, RegisterMode::Existing);
    }
    r_ind++;
  }

  // Set up CSV files for wititing the data.
  std::ofstream pose_file, cov_file, info_file, exp_file;
  fs::path results_path{results_dir};
  fs::create_directory(results_path);
  pose_file.open(fs::path{results_path / "poses.csv"});
  cov_file.open(fs::path{results_path / "covariances.csv"});
  info_file.open(fs::path{results_path / "info.csv"});
  exp_file.open(fs::path{results_path / "exp.csv"});

  pose_file << "timestamp,live_id,priv_id,t_live_priv\n";
  cov_file << "timestamp,live_id,priv_id,cov_set,cov_live_priv\n";
  info_file << "timestamp,live_id,priv_id,success,inliers_rgb,inliers_gr,inliers_cc,window_temporal_depth,window_num_vertices,comp_time,exp\n";
  exp_file << "live_id,exp_run_ids\n";

  r_ind = 0;
  for (const auto& run : graph->runs()) {

    if (r_ind > 0) {
      int num_fail = 0;
      int num_multiple_exp = 0;
      int num_cov_not_set = 0;
      int total_inliers = 0;
      float total_comp_time = 0.0;
      int num_vertices = run.second->vertices().size();

      for (int v_ind = 0; v_ind < num_vertices; v_ind++) {

        try {
          auto v = graph->at(VertexId(r_ind, v_ind));
        
          try {
            v->load(stream_name_loc);
            auto msg = v->retrieveKeyframeData<vtr_messages::msg::LocalizationStatus>(
              stream_name_loc);

            try {
              v->load(stream_name_exp);
              auto msg_exp = v->retrieveKeyframeData<vtr_messages::msg::ExpRecogStatus>(
                stream_name_exp);

              exp_file << msg_exp->query_id;

              int num_exp = msg_exp ->recommended_ids.size();

              for (int j = 0; j < num_exp; j ++) {
                exp_file << "," << msg_exp->recommended_ids[j];    
              }
       
              exp_file << "\n";  

              if (num_exp > 1) {
                num_multiple_exp++;
                LOG(ERROR) << "Recommending " << num_exp << " experiences";
                LOG(ERROR) << "EXPERIENCE" << msg_exp->recommended_ids.back();
                LOG(ERROR) << "EXPERIENCES: " << msg_exp->recommended_ids;
              }
            } catch (const std::exception& e){
              LOG(ERROR) << "COULD NOT LOAD EXP MESSAGE";
              *num_fail_read_exp_all++;
            }

            pose_file << msg->keyframe_time << "," 
                      << msg->query_id << "," 
                      << msg->map_id << ","
                      << msg->t_query_map.xi[0] << ","
                      << msg->t_query_map.xi[1] << ","
                      << msg->t_query_map.xi[2] << ","
                      << msg->t_query_map.xi[3] << ","
                      << msg->t_query_map.xi[4] << ","
                      << msg->t_query_map.xi[5] << "\n"; 

            cov_file << msg->keyframe_time << "," 
                     << msg->query_id << "," 
                     << msg->map_id << ","
                     << msg->t_query_map.cov_set;

            for (int i = 0; i < 36; i ++) {
              cov_file << "," << msg->t_query_map.cov[i];    
            }
     
            cov_file << "\n";  

            info_file << msg->keyframe_time << "," 
                      << msg->query_id << "," 
                      << msg->map_id << "," 
                      << msg->success << "," 
                      << msg->inlier_channel_matches[0] << ","
                      << msg->inlier_channel_matches[1] << ","
                      << msg->inlier_channel_matches[2] << "," 
                      << msg->window_temporal_depth << ","
                      << msg->window_num_vertices << ","
                      << msg->localization_computation_time_ms << "\n";

            if (!msg->success) {
              num_fail++;
            }
            if (!msg->t_query_map.cov_set) {
              num_cov_not_set++;
            }
            total_inliers += msg->inlier_channel_matches[0];
            total_comp_time += msg->localization_computation_time_ms;

          } catch (const std::exception& e){
            LOG(ERROR) << "COULD NOT LOAD LOC STATUS MSG, run: " << r_ind << ", vert: " << v_ind;
            continue;
          }

        } catch (const std::exception& e){
          LOG(ERROR) << "COULD NOT LOAD VERTEX, run: " << r_ind << ", vert: " << v_ind;
          continue;
        }
      }
      
      LOG(INFO) << "Num failed loc: " << num_fail;
      LOG(INFO) << "Num multiple exp: " << num_multiple_exp;
      LOG(INFO) << "Num cov not set: " << num_cov_not_set;
      LOG(INFO) << "Avg. num inliers: " << float(total_inliers) / num_vertices;
      LOG(INFO) << "Avg. comp_time: " << float(total_comp_time) / num_vertices;

      *num_fail_loc_all += num_fail;
      *num_fail_exp_all += num_multiple_exp;

      pose_file.close();
      cov_file.close();
      info_file.close();
      exp_file.close();
    }
    
  r_ind++;
  }
}

// Run this twice. Second time tests retrieval from disk.
int main(int argc, char** argv) {

  std::string path_name = argv[argc-2];
  int num_runs = atoi(argv[argc-1]);

  LOG(INFO) << "Path name: " << path_name;
  LOG(INFO) << "Num runs: " << num_runs;

  int num_fail_loc_all = 0;
  int num_fail_exp_all = 0;
  int num_fail_read_exp_all = 0;

  std::vector<int> run_ids{8, 35, 174, 235, 328, 538, 587};
  
  for(int i = 0; i <= run_ids.size(); i++) {
    std::stringstream graph_dir;
    std::stringstream results_dir;
    graph_dir << path_name << "/graph.index/repeats/" << run_ids[i] << "/graph.index";
    results_dir << path_name << "/graph.index/repeats/" << run_ids[i] << "/results";

    LOG(INFO) << "RUN: " << run_ids[i];

    ReadLocalizationResults(graph_dir.str(), results_dir.str(),
                            &num_fail_loc_all, &num_fail_exp_all, 
                            &num_fail_read_exp_all); 
  }

  LOG(INFO) << "Total failed loc: " << num_fail_loc_all;
  LOG(INFO) << "Total failed exp: " << num_fail_exp_all;
  LOG(INFO) << "Total failed read exp: " << num_fail_read_exp_all; 
}