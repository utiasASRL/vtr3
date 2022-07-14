// Initialize planner configs:
// Replace this file with integration into VT&R configs

#pragma once 

class CBITConfig {
    public:
        CBITConfig() = default; 

        // Environment
        double obs_padding = 2.0;
        int curv_to_euclid_discretization = 20;
        double sliding_window_width = 15.0;
        double sliding_window_freespace_padding = 0.5;
        double corridor_resolution = 0.05;
        double state_update_freq = 1; // In Hz
        bool update_state = true;
        int rand_seed = 1;

        // ROC
        double roc_lookahead = 5.0;
        int roc_discretization = 40;
        double roc_q_tolerance = 0.001;

        // Planner Tuning Params
        int initial_samples = 250;
        int batch_samples = 100;
        int pre_seed_resolution = 0.5;
        double alpha = 0.5;
        double q_max = 2.5;
        int frame_interval = 50;
        int iter_max = 2000;
        double eta = 1.1;
        double rad_m_exhange = 1.00;
        double initial_exp_rad = 1.00;

        // Misc
        bool incremental_plotting = true;
        bool plotting = true;   

};