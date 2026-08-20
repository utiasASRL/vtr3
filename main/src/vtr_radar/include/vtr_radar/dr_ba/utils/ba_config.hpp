// ba_config.hpp
#pragma once

#include <string>
#include <filesystem>

namespace fs = std::filesystem;

namespace ba {

struct OptimizationOptions {
    int max_iterations = 20;
    double convergence_tol = 1e-3;
    double alpha = 0.5;        // step size
    int max_cost_increases = 3; // max number of consecutive iterations with cost increase before stopping
    double meas_std = 1.0;       // intensity units
    bool use_pose_prior = false;
    double rel_pose_prior_translation_std = 0.1; // meters
    double rel_pose_prior_rotation_std = 5.0;    // degrees
    double range_factor = 0.0;   // factor to scale range uncertainty to intensity uncertainty
    bool use_cumul_thresh = true;
    double cumul_thresh = 1.0;   // threshold to ignore measurements with too high a cumulative return
    double zero_thresh = 1.0;    // max cumul return threshold to consider a measurement as zero return
    int num_coarse_iterations = 5; // number of initial iterations with higher downsampling
    double coarse_downsample = 0.2; // downsampling factor for coarse iterations
    double refine_downsample = 1.0; // downsampling factor for refinement iterations
    double tile_size = 0.0;     // meters, size of tiles to process separately
    int max_loaded_scans = 0;   // max number of scans to keep loaded in memory at once (<1 all)
};

struct FrameProcessingOptions {
    std::string input_type = "local_maps"; // 'scans' or 'local_maps' with both being images (for now)
    double local_map_res = 0.1;  // m/pixel (only relevant if input_type is 'local_map')
    double max_dist = 80.0; // meters
    double gauss_blur_sigma = 3.0; // pixels
    bool adaptive_blur = true; // adaptively choose Gaussian blur sigma based on scan content
    double max_blur_sigma = 15.0; // max sigma to use for adaptive blur
    double min_int_val_tol = 0.5; // intensity units
    double min_percent_nonzero = 0.3; // percent
};

struct BAOptions {
    double voxel_res = 1.0;        // meters
    std::string seq_id;
    bool save_H = false;   // whether to save the visualization of the Hessian, overwrittten each iteration if true
    std::string solver = "drba";  // 'combined' or 'drba'
    std::string init_poses = "gt";  // 'pogo' or 'gt', 'dro'
    double init_translation_std = 0.0; // m
    double init_rotation_std = 0.0;    // degrees
    std::vector<std::pair<int, int>> frame_ranges = {{0, -1}}; // pairs of (start_frame, end_frame) to consider for mapping, -1 for end_frame means last frame

    // Keyframing
    double max_kf_dist = 2.0;    // meters
    double max_kf_rot = 10.0;    // degrees
    bool fix_first_scan = true;

    // Frame processing
    FrameProcessingOptions frame_processing_opts;

    // BA optimization
    OptimizationOptions optimization_opts;
};

struct MappingOptions {
    double voxel_res = 1.0;        // meters
    std::string seq_id;
    std::string pose_source = "gt"; // 'estimate', 'gt', 'pogo', 'dro'
    double init_translation_std = 0.0; // m
    double init_rotation_std = 0.0;    // degrees
    fs::path estimate_location;
    std::vector<std::pair<int, int>> frame_ranges = {{0, -1}}; // pairs of (start_frame, end_frame) to consider for mapping, -1 for end_frame means last frame

    // Keyframing
    double max_kf_dist = 2.0;    // meters
    double max_kf_rot = 10.0;    // degrees
    bool fix_first_scan = true;

    // Frame processing
    FrameProcessingOptions frame_processing_opts;

    // Map optimization
    OptimizationOptions optimization_opts;
};

struct LocalizationOptions {
    std::string seq_id;
    std::string map_seq_id;
    fs::path map_location;
    int start_frame = 0;
    int end_frame = -1; // -1 means last frame
    bool use_odometry_prior = false;
    double odom_translation_std = 0.1; // m
    double odom_rotation_std = 0.1;    // degrees

    // Frame processing
    FrameProcessingOptions frame_processing_opts;

    // Localization optimization
    OptimizationOptions optimization_opts;
};

struct Options {
    int num_threads = 1;
    int seed = -1; // random seed for reproducibility, <0 for random

    // Data paths
    fs::path data_path;
    fs::path meas_path;

    // Output parameters
    fs::path output_path;
    bool visualize_result = true;
    bool save_result = true;

    // BA parameters
    BAOptions ba_opts;

    // Mapping parameters
    MappingOptions map_opts;

    // Localization parameters
    LocalizationOptions loc_opts;
};


} // namespace ba