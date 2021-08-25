#include <filesystem>
// #include <c++/8/fstream>
#include <fstream>
#include <iostream>
#include <random>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vtr_logging/logging_init.hpp>
#include <vtr_messages/msg/rig_landmarks.hpp>
#include <vtr_messages/msg/rig_observations.hpp>
#include <vtr_messages/msg/rig_images.hpp>
#include <vtr_messages/msg/rig_calibration.hpp>
#include <vtr_messages/msg/localization_status.hpp>
#include <vtr_messages/msg/exp_recog_status.hpp>
#include <vtr_messages/msg/matches.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#include <vtr_pose_graph/path/path.hpp>
#include "rclcpp/rclcpp.hpp"

#include <vtr_common/timing/time_utils.hpp>
#include <vtr_common/utils/filesystem.hpp>

namespace fs = std::filesystem;
using namespace vtr::pose_graph;

cv::Mat wrapImage(const vtr_messages::msg::Image &asrl_image) {
  const auto & data = asrl_image.data;

  //assert(data != nullptr);

  // Convert to opencv
  uint32_t width = asrl_image.width;
  uint32_t height =  asrl_image.height;
  std::string encoding = asrl_image.encoding;

  if(encoding == "mono8") {
    return cv::Mat(cv::Size(width,height),CV_8UC1,(void*)data.data());
  } else if (encoding == "bgr8") {
    return  cv::Mat(cv::Size(width,height),CV_8UC3,(void*)data.data());
  } else {
    return cv::Mat();
  }
}

cv::Mat setupDisplayImage(cv::Mat input_image) {
  // create a visualization image to draw on.
  cv::Mat display_image;
  if (input_image.type() == CV_8UC1) {
    cv::cvtColor(input_image, display_image, cv::COLOR_GRAY2RGB);
  } else if (input_image.type() == CV_16S) {
    input_image.convertTo(display_image, CV_8U, 255/(48*16.));
  } else {
    display_image = input_image.clone();
  }
  return display_image;
}

void ReadLocalizationResults(std::string graph_dir, std::string results_dir,
                             std::string image_dir, int* num_fail_loc_all, 
                             int* num_fail_read_obs_all, int vis) {

  bool store_img = false;
  if (vis > 0) {
    store_img = true;
  }

  // Load the graph
  std::shared_ptr<RCGraph> graph;
  graph = vtr::pose_graph::RCGraph::LoadOrCreate(graph_dir, 0);
  LOG(INFO) << "Loaded graph has " << graph->vertices()->size() << " vertices";

  // Register the stream so we can read messages from it
  std::string stream_name_loc = "results_localization";
  std::string stream_name_obs = "front_xb3_observations";
  std::string stream_name_match = "front_xb3_landmarks_matches";

  int r_ind = 0;
  for (const auto& run : graph->runs()) {
    // We are iterating over the teach and one repeat, run_000000 and run_000001
    if (r_ind == 1) { 
      run.second->registerVertexStream<vtr_messages::msg::LocalizationStatus>(
        stream_name_loc, true, RegisterMode::Existing);
      run.second->registerVertexStream<vtr_messages::msg::RigObservations>(
        stream_name_obs, true, RegisterMode::Existing);
      LOG(ERROR) << "here0";
      run.second->registerVertexStream<vtr_messages::msg::Matches>(
        stream_name_match, true, RegisterMode::Existing);
      LOG(ERROR) << "here1";
    }
    r_ind++;
  }

  // Set up CSV files for wititing the data.
  std::ofstream obs_file;
  fs::path results_path{results_dir};
  fs::create_directory(results_path);
  fs::path results_img_path{fs::path{results_path / "images"}};
  fs::create_directory(results_img_path);
  obs_file.open(fs::path{results_path / "obs.csv"});
  obs_file << "timestamp,u,v\n";

  r_ind = 0;
  for (const auto& run : graph->runs()) {

    if (r_ind == 1) {
      int num_fail = 0;
      int num_multiple_exp = 0;
      int num_cov_not_set = 0;
      int total_inliers = 0;
      float total_comp_time = 0.0;
      int num_vertices = run.second->vertices().size();
      int start_index = 0;
      int stop_index = 30000;
      int num_diff_im = 0;

      for (int v_ind = 0; v_ind < num_vertices; v_ind++) {

        // try {
          auto v = graph->at(VertexId(r_ind, v_ind));
        
          // try {
            v->load(stream_name_loc);
            auto msg = v->retrieveKeyframeData<vtr_messages::msg::LocalizationStatus>(
              stream_name_loc);

            cv::Mat img;
            if (store_img) {
              /// Playback images
              vtr::storage::DataStreamReader<vtr_messages::msg::RigImages, vtr_messages::msg::RigCalibration> stereo_stream(
                  vtr::common::utils::expand_user(vtr::common::utils::expand_env(image_dir)),"front_xb3");             

              bool seek_success = stereo_stream.seekByIndex(static_cast<int32_t>(start_index));
              if (!seek_success) {
                LOG(ERROR) << "Seek failed!";
                return;
              }

              vtr_messages::msg::RigImages rig_images;
              bool found_img = false;
              int idx = start_index;
              while (idx < stop_index) {
                auto storage_msg = stereo_stream.readNextFromSeek();
                if (!storage_msg) {
                  LOG(ERROR) << "Storage msg is nullptr! " << idx;
                  idx++;
                  continue;
                }
                rig_images = storage_msg->template get<vtr_messages::msg::RigImages>();
                auto timestamp = rig_images.channels[0].cameras[0].stamp.nanoseconds_since_epoch;
                if (timestamp >= msg->keyframe_time) {
                  int diff1 = timestamp - msg->keyframe_time;
                  if (diff1 > 0) {
                    LOG(INFO) << "Diff: " << diff1;
                    num_diff_im++;
                  }
                  found_img = true;
                  start_index = idx; 
                  break;
                }
                idx++;
              }

              if (!found_img) {
                LOG(ERROR) << "Couldn't find matching image";
                continue;
              }

              // auto img_msg = vis_msg->channels[0].cameras[0];
              auto input_image = wrapImage(rig_images.channels[0].cameras[0]);
              img = setupDisplayImage(input_image);
            }
              
            try {
              v->load(stream_name_match);
              auto matches_msg = v->retrieveKeyframeData<vtr_messages::msg::Matches>(
                stream_name_match);

              v->load(stream_name_obs);
              auto obs_msg = v->retrieveKeyframeData<vtr_messages::msg::RigObservations>(
                stream_name_obs);

              auto rgb_channel_obs = obs_msg->channels[0].cameras[0];
              auto keypoints = rgb_channel_obs.keypoints;
              auto landmarks_matches = rgb_channel_obs.landmarks;
              auto inlier_matches = matches_msg->matches; 

              // LOG(INFO) << "Num inliers: " << msg->inlier_channel_matches[0];
              LOG(INFO) << "Num keypoints: " << keypoints.size();
              // LOG(INFO) << "Num landmark matches: " << landmarks_matches.size();
              LOG(INFO) << "Num landmark inliers: " << inlier_matches.size();
 

              // Store the keypoints in a csv file.
              obs_file << msg->keyframe_time << "," 
                       << msg->query_id << "," 
                       << msg->map_id << "," 
                       << msg->success << "," 
                       << msg->inlier_channel_matches[0];
              
              for (int k_ind = 0; k_ind < keypoints.size(); k_ind++) {

                // Check if the keypoint corrsponds to an inlier match.
                auto obs_match = landmarks_matches[k_ind];

                for (int m_ind = 0; m_ind < inlier_matches.size(); m_ind++) {
                  // LOG(INFO) << "Obs: " << obs_match.from_id.idx;
                  // LOG(INFO) << "Inl: " << inlier_matches[m_ind].from_id;
                  if (obs_match.from_id.idx == inlier_matches[m_ind].from_id.idx) {
                    obs_file << "," << keypoints[k_ind].position.x
                             << "," << keypoints[k_ind].position.y;

                    if (store_img) {
                      // Draw keypoints in image.
                      cv::Point img_point(keypoints[k_ind].position.x, keypoints[k_ind].position.y);
                      cv::Scalar ptColor(0, 0, 255);
                      cv::circle(img, img_point, 3.0, ptColor, 2.0);
                    }
                  }
                }
              }

              obs_file << "\n";  

              if (store_img) {
                std::stringstream img_file;
                img_file << results_img_path.u8string() << "/" << v_ind << ".png";
                // fs::path img_path = fs::path{results_img_path / std::to_string(v_ind) /".png"};
                // std::string img_path_str{img_path.u8string()};
                cv::imwrite(img_file.str(), img);
              }

            } catch (const std::exception& e){
              LOG(ERROR) << "COULD NOT LOAD OBSERVATION MESSAGE: " << v_ind << "/" << num_vertices;
              *num_fail_read_obs_all++;
            }

            if (!msg->success) {
              num_fail++;
            }
            total_inliers += msg->inlier_channel_matches[0];
            LOG(INFO) << "Number images not matched: " << num_diff_im;

        //   } catch (const std::exception& e){
        //     LOG(ERROR) << "COULD NOT LOAD LOC STATUS MSG, run: " << r_ind << ", vert: " << v_ind;
        //     continue;
        //   }

        // } catch (const std::exception& e){
        //   LOG(ERROR) << "COULD NOT LOAD VERTEX, run: " << r_ind << ", vert: " << v_ind;
        //   continue;
        // }
      }
      
      LOG(INFO) << "Num failed loc: " << num_fail;
      LOG(INFO) << "Avg. num inliers: " << float(total_inliers) / num_vertices;

      *num_fail_loc_all += num_fail;

      obs_file.close();
    }
    
  r_ind++;
  }
}

// Run this twice. Second time tests retrieval from disk.
int main(int argc, char** argv) {

  std::string path_name = argv[argc-5];
  std::string path_name_im = argv[argc-4];
  int start = atoi(argv[argc-3]);
  int end = atoi(argv[argc-2]);
  int vis = atoi(argv[argc-1]);

  LOG(INFO) << "Path name: " << path_name;
  LOG(INFO) << "Path name images: " << path_name_im;
  LOG(INFO) << "Start: " << start;
  LOG(INFO) << "End: " << end;

  int num_fail_loc_all = 0;
  int num_fail_read_obs_all = 0;
  
  for(int i = start; i <= end; i++) {
    std::stringstream graph_dir;
    std::stringstream results_dir;
    std::stringstream image_dir;
    graph_dir << path_name << "/graph.index/repeats/" << i << "/graph.index";
    results_dir << path_name << "/graph.index/repeats/" << i << "/results";
    if (i > 9) {
      image_dir << path_name_im << "/front-xb3/run_0000" << i;
    } else {
      image_dir << path_name_im << "/front-xb3/run_00000" << i;
    }

    LOG(INFO) << "RUN: " << i;

    ReadLocalizationResults(graph_dir.str(), results_dir.str(), image_dir.str(),
                            &num_fail_loc_all, &num_fail_read_obs_all, vis); 
  }

  LOG(INFO) << "Total failed loc: " << num_fail_loc_all;
  LOG(INFO) << "Total failed read exp: " << num_fail_read_obs_all; 
}