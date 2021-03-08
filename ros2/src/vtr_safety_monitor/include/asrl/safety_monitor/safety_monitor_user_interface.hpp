#ifndef stdLLASRL_SAFETY_MONITOR_USER_INTERFACE_HPP
#define ASRL_SAFETY_MONITOR_USER_INTERFACE_HPP

#include <ros/ros.h>

#include <asrl/rosutil/image_utilities.hpp>

//For outputting video cv::imshow
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// For action enumerations
#include <asrl/safety_monitor/safety_monitor_node.hpp>


#include <asrl__control__path_tracker/StatusOut.h>
#include <asrl__vehicle_safety__safety_monitor/MonitorDebug.h>

#include <asrl/rosutil/transformation_utilities.hpp>

namespace asrl {
  namespace safetyMonitor {

    class user_interface
    {
    public :
        user_interface(ros::NodeHandle nh);
        void spin(void);

        bool produce_output();
        void initialize_update_timer();
        void pathTrackerStatusCallback(const asrl__control__path_tracker::StatusOutConstPtr & status);
        void monitorDebugCallback(const asrl__vehicle_safety__safety_monitor::MonitorDebugConstPtr & debug);
        ros::Timer generateImageTimer;
        void generateImageCallback(const ros::TimerEvent& timerEvent);

    private :
        ros::NodeHandle nodeHandle_;

        void plot_ticks(cv::Mat & cv_img,
                       const float & path_scaling,
                       const cv::Rect & rect,
                       const cv::Point robot_origin);

        void plot_pose_sequence(cv::Mat & cv_img,
                       const cv::Point & robot_origin,
                       const float & path_scaling,
                       const std::vector<geometry_msgs::Pose> & pose_sequence,
                       const cv::Scalar & color,
                       const cv::Rect & rect,
                       bool flg_showVertices,
                       bool flg_showLines);

        void plot_pose_3sig_boundary(cv::Mat & cv_img,
                       const cv::Point & robot_origin,
                       const float & path_scaling,
                       const std::vector<geometry_msgs::Pose> & wc1_sequence,
                       const std::vector<geometry_msgs::Pose> & wc2_sequence,
                       const cv::Scalar & color);

        void plot_robot(cv::Mat & cv_img, const cv::Point & robot_origin, const cv::Scalar color);

        void write_speed(cv::Mat & cv_img, const cv::Point & text_origin, std::string & vehicle_speed);

        void plot_debug_box(cv::Mat & cv_img,
                        const cv::Point & box_tl,
                        const cv::Point & box_br,
                        std::string & signal_monitor_name,
                        int & signal_monitor_action);

        // Subscriber for path tracking plot
        ros::Subscriber pathTrackerStatus_;
        std::vector<geometry_msgs::Pose> desired_path_;
        std::vector<geometry_msgs::Pose> desired_pose_tol_pos_;
        std::vector<geometry_msgs::Pose> desired_pose_tol_neg_;
        std::vector<geometry_msgs::Pose> predicted_path_;
        std::vector<geometry_msgs::Pose> wc1_predicted_poses_;
        std::vector<geometry_msgs::Pose> wc2_predicted_poses_;
        float current_speed_;

        // Subscriber for monitor debug information
        ros::Subscriber monitorDebugSubscriber_;
        std::vector<std::string> limiting_signal_monitor_names_;
        std::vector<int> limiting_signal_monitor_actions_;

        ros::Publisher visImagePublisher_;

        double default_plot_height_;

    };

  } // safetyMonitor
} // asrl

#endif  // ASRL_SAFETY_MONITOR_USER_INTERFACE_HPP
