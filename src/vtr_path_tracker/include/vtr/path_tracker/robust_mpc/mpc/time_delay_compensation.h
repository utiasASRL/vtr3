#pragma once


#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wreturn-type"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

#include <ros/ros.h>

#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop

#include <asrl/common/logging.hpp>
#include <yaml-cpp/yaml.h>
#include <deque>

namespace vtr {
  namespace path_tracker {

    class MpcTimeDelayComp
    {
    private:
        // Struct to hold entries
        typedef struct {
            ros::Time ctrl_time;
            double v_cmd;
            double w_cmd;
        } cmd_hist_entry;

        // Command History
        std::deque< cmd_hist_entry > cmd_hist;

    public:

        MpcTimeDelayComp();
        ~MpcTimeDelayComp();

        // Functions
        void clear_hist(void);
        bool add_hist_entry(const float & v_cmd, const float & w_cmd, const ros::Time & ctrl_time);
        bool get_cmd_list(const ros::Time & t_1, const ros::Time & t_2, std::vector< float > & v_cmd_vec, std::vector< float > & w_cmd_vec, std::vector< float > & dt_time_vec);
        bool get_avg_cmd(const ros::Time & t_1, const ros::Time & t_2, float & v_cmd_avg, float & w_cmd_avg);
        bool del_hist_older_than(const ros::Time & t_1);

        // Utilities
        int get_size(void);

    };

  } // path_tracker
} // asrl

