#include <asrl/safety_monitor/safety_monitor_user_interface.hpp>



asrl::safetyMonitor::user_interface::user_interface(ros::NodeHandle nh) :
  nodeHandle_(nh)
{
    pathTrackerStatus_ = nodeHandle_.subscribe<asrl__control__path_tracker::StatusOut>("in/path_tracker_status", 100, &user_interface::pathTrackerStatusCallback, this);
    monitorDebugSubscriber_ = nodeHandle_.subscribe<asrl__vehicle_safety__safety_monitor::MonitorDebug>("in/monitor_debug", 100, &user_interface::monitorDebugCallback, this);
    generateImageTimer = nodeHandle_.createTimer(ros::Duration(0.2), &user_interface::generateImageCallback, this, false);

    visImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>("out/system_status", 1, true);

    desired_path_.clear();
    predicted_path_.clear();
    wc1_predicted_poses_.clear();
    wc2_predicted_poses_.clear();
    current_speed_ = 0;

    limiting_signal_monitor_names_.clear();
    limiting_signal_monitor_actions_.clear();

    asrl::rosutil::param<double>(nodeHandle_,"default_plot_height_",default_plot_height_,3.0);
}

void asrl::safetyMonitor::user_interface::spin(){
  ros::spin();
  return;
}

void getTfQuaternion(const geometry_msgs::Pose_<std::allocator<void> >& pose, tf::Quaternion& q){
    q.setX(pose.orientation.x);
    q.setY(pose.orientation.y);
    q.setZ(pose.orientation.z);
    q.setW(pose.orientation.w);
}

void asrl::safetyMonitor::user_interface::pathTrackerStatusCallback(const asrl__control__path_tracker::StatusOutConstPtr & status){

    desired_path_ = status->desired_poses;
    //desired_pose_tol_pos_ = status->desired_pose_tol_pos;
    //desired_pose_tol_neg_ = status->desired_pose_tol_neg;
    predicted_path_ = status->nominal_predicted_poses;
    wc1_predicted_poses_ = status->wc1_predicted_poses;
    wc2_predicted_poses_ = status->wc2_predicted_poses;
    current_speed_ = status->commanded_linear_speed;


    desired_pose_tol_pos_.clear();
    desired_pose_tol_neg_.clear();

    // Check if the msg includes tolerance limits
    if (status->desired_poses.size() == status->desired_pose_tol_pos.size()){
        geometry_msgs::Pose temp;
        tf::Quaternion q_0_k_0(0,0,0,0);

        for (int i=0; i < status->desired_poses.size(); i++){
            getTfQuaternion(desired_path_[i],q_0_k_0);
            tf::Transform C_0_k(q_0_k_0);
            tf::Point des_pose(desired_path_[i].position.x,desired_path_[i].position.y,0);

            tf::Point tol_pos(0,status->desired_pose_tol_pos[i],0);
            tf::Point tol_neg(0,status->desired_pose_tol_neg[i],0);

            tf::Point tol_pose_pos = des_pose + C_0_k*tol_pos;
            tf::Point tol_pose_neg = des_pose + C_0_k*tol_neg;

            desired_pose_tol_pos_.push_back(desired_path_[i]);
            desired_pose_tol_pos_[i].position.x = tol_pose_pos.getX();
            desired_pose_tol_pos_[i].position.y = tol_pose_pos.getY();

            desired_pose_tol_neg_.push_back(desired_path_[i]);
            desired_pose_tol_neg_[i].position.x = tol_pose_neg.getX();
            desired_pose_tol_neg_[i].position.y = tol_pose_neg.getY();


        }
   }

}


void asrl::safetyMonitor::user_interface::monitorDebugCallback(const asrl__vehicle_safety__safety_monitor::MonitorDebugConstPtr & debug){

    limiting_signal_monitor_names_ = debug->limiting_signal_monitor_names;
    limiting_signal_monitor_actions_ = debug->limiting_signal_monitor_actions;

}

void asrl::safetyMonitor::user_interface::generateImageCallback(const ros::TimerEvent& timerEvent){

    /****
    Define colors and window sizes
    ****/

    int window_size_width = 400;
    int window_size_height = 800;
    int border_depth = 25;
    cv::Scalar bkgnd_color(240,240,240);
    cv::Scalar pt_plot_color(220,220,220);
    cv::Scalar text_color(0,0,0);
    cv::Scalar grn_color(240,240,240);
    cv::Scalar orng_color(240,240,240);
    cv::Scalar red_color(240,240,240);

    cv::Mat cv_img(window_size_height, window_size_width, CV_8UC3, bkgnd_color);
    cv::Mat cv_img2;
    sensor_msgs::Image::Ptr ros_img(new sensor_msgs::Image());


    asrl::rosutil::initImageView(cv_img2, *ros_img, cv_img.cols, cv_img.rows, CV_8UC3);

    /****
    Draw path tracker plots
    ****/

    int pt_plot_height = 400;
    int pt_plot_width = window_size_width - 2*border_depth;
    float path_scaling = (float) pt_plot_height / default_plot_height_; // pixels / meters



    /** Window for plot **/
    cv::Point pt_plot_topLeft(border_depth,border_depth);
    cv::Point pt_plot_botRight(border_depth+pt_plot_width, border_depth+pt_plot_height);
    cv::Point plot_center(border_depth+pt_plot_width/2, border_depth+pt_plot_height/2);

    rectangle(cv_img, pt_plot_topLeft,pt_plot_botRight, pt_plot_color, -1, 8);

    cv::Rect pt_plot_window(pt_plot_topLeft,pt_plot_botRight);
    plot_ticks(cv_img, path_scaling, pt_plot_window, plot_center);

    /** Robot and predicted poses **/
    cv::Point robot_origin_m(0,0);
    cv::Scalar robot_color(0,0,240);
    cv::Scalar desired_path_color(50,50,50);
    cv::Scalar nominal_pred_color(0,0,0);
    cv::Scalar sig_pred_color(100,0,100);


    if (predicted_path_.size() > 0){
        robot_origin_m = cv::Point(predicted_path_[0].position.y*path_scaling, predicted_path_[0].position.x*path_scaling);
    }

    plot_pose_3sig_boundary(cv_img, plot_center+robot_origin_m, path_scaling, wc1_predicted_poses_, wc2_predicted_poses_, sig_pred_color);
    bool flg_show_vertices = true;
    bool flg_show_lines = false;
    plot_pose_sequence(cv_img, plot_center+robot_origin_m, path_scaling, desired_path_, desired_path_color, pt_plot_window, flg_show_vertices, flg_show_lines);
    flg_show_vertices = false;
    flg_show_lines = true;

    if (desired_pose_tol_pos_.size() == desired_path_.size()){
        plot_pose_sequence(cv_img, plot_center+robot_origin_m, path_scaling, desired_pose_tol_pos_, desired_path_color, pt_plot_window, flg_show_vertices, flg_show_lines);
        plot_pose_sequence(cv_img, plot_center+robot_origin_m, path_scaling, desired_pose_tol_neg_, desired_path_color, pt_plot_window, flg_show_vertices, flg_show_lines);
    }

    plot_pose_sequence(cv_img, plot_center+robot_origin_m, path_scaling, predicted_path_, nominal_pred_color, pt_plot_window, flg_show_vertices, flg_show_lines);
    plot_robot(cv_img, plot_center, robot_color);

    char vehicle_speed[256];
    std::sprintf(vehicle_speed,"%4.2f m/s",fabs(current_speed_));
    std::string vehicle_speed_str = vehicle_speed;
    cv::Point text_origin(border_depth+pt_plot_width*6/10, border_depth+pt_plot_height*9.5/10);

    write_speed(cv_img, text_origin, vehicle_speed_str);

    /****
    Show Active Signal Monitors
    ****/

    int debug_box_spacing = 10;
    int debug_box_total_space = window_size_height - 3*border_depth - pt_plot_height;
    int max_debug_box_height = 50;
    int min_debug_box_height = 20;
    int num_debug_boxes = limiting_signal_monitor_names_.size();
    int debug_box_height = 0;

    if (num_debug_boxes > 0){
        int total_debug_box_gap_space = debug_box_spacing*(num_debug_boxes-1);
        int nominal_debug_box_height = (debug_box_total_space-total_debug_box_gap_space)/num_debug_boxes;
        debug_box_height = std::min(max_debug_box_height, std::max(min_debug_box_height, nominal_debug_box_height));
    } else {
        debug_box_height = max_debug_box_height;
        limiting_signal_monitor_names_.push_back("All OK");
        limiting_signal_monitor_actions_.push_back(CONTINUE);
        num_debug_boxes = 1;
    }

    cv::Point box_tl(border_depth, pt_plot_botRight.y + border_depth);

    for (int i = 0; i < num_debug_boxes; i++){
        cv::Point box_br = box_tl + cv::Point(pt_plot_width, debug_box_height);
        plot_debug_box(cv_img, box_tl, box_br, limiting_signal_monitor_names_[i], limiting_signal_monitor_actions_[i]);

        box_tl = cv::Point(box_tl.x, box_tl.y+debug_box_height+debug_box_spacing);

    }

    // Publish the image
    cv_img.copyTo(cv_img2);
    ros_img->encoding = sensor_msgs::image_encodings::RGB8;
    visImagePublisher_.publish(ros_img);

}

void asrl::safetyMonitor::user_interface::plot_robot(cv::Mat & cv_img, const cv::Point & robot_origin, const cv::Scalar color){

    cv::Point robot_outline[1][3];
    robot_outline[0][0] = robot_origin + cv::Point(-10,+10);
    robot_outline[0][1] = robot_origin + cv::Point(+10,+10);
    robot_outline[0][2] = robot_origin + cv::Point(0,-20);

    const cv::Point* ppt[1] = { robot_outline[0] };
    int npt[] = { 3 };
    cv::fillPoly(cv_img, ppt, npt, 1, color, 8);

}

void asrl::safetyMonitor::user_interface::plot_ticks(cv::Mat & cv_img,
                                                   const float & path_scaling,
                                                   const cv::Rect & rect,
                                                   const cv::Point robot_origin)
{

    int max_horiz_lines = 20;
    int max_vert_lines = 20;
    float grid_spacing = 0.5;
    cv::Scalar color(10,10,10);
    int thickness = 1;
    int lineType = 8;


    cv::Point tl = rect.tl();
    cv::Point br = rect.br();
    cv::Point start, end;

    // Horizontal
    int left_edge = tl.x;
    int right_edge = br.x;

    for (int i = 0; i < max_horiz_lines; i++){

        int vert_position = robot_origin.y + i*grid_spacing*path_scaling;

        start = cv::Point(left_edge, vert_position);
        end   = cv::Point(right_edge, vert_position);

        if (cv::Point(robot_origin.x, vert_position).inside(rect)){
            line( cv_img, start, end, color, thickness, lineType );
        }

        vert_position = robot_origin.y - i*grid_spacing*path_scaling;

        start = cv::Point(left_edge, vert_position);
        end   = cv::Point(right_edge, vert_position);

        if (cv::Point(robot_origin.x, vert_position).inside(rect)){
            line( cv_img, start, end, color, thickness, lineType );
        }

    }

    // Vertical
    int top_edge = tl.y;
    int bottom_edge = br.y;

    for (int i = 0; i < max_vert_lines; i++){

        int horiz_position = robot_origin.x + i*grid_spacing*path_scaling;

        start = cv::Point(horiz_position, top_edge);
        end   = cv::Point(horiz_position, bottom_edge);

        if (cv::Point(horiz_position, robot_origin.y).inside(rect)){
            line( cv_img, start, end, color, thickness, lineType );
        }

        horiz_position = robot_origin.x - i*grid_spacing*path_scaling;

        start = cv::Point(horiz_position, top_edge);
        end   = cv::Point(horiz_position, bottom_edge);

        if (cv::Point(horiz_position, robot_origin.y).inside(rect)){
            line( cv_img, start, end, color, thickness, lineType );
        }

    }

}

void asrl::safetyMonitor::user_interface::plot_pose_sequence(
                                                    cv::Mat & cv_img,
                                                    const cv::Point & robot_origin,
                                                    const float & path_scaling,
                                                    const std::vector<geometry_msgs::Pose> & pose_sequence,
                                                    const cv::Scalar & color,
                                                    const cv::Rect & rect,
                                                    bool flg_showVertices,
                                                    bool flg_showLines){

    if (pose_sequence.size() < 1){
        return;
    }

    int thickness = 2;
    int lineType = 8;
    cv::Point start, end;
    for (int i = 0; i < pose_sequence.size()-1; i++){
        start = cv::Point(robot_origin.x - pose_sequence[i].position.y*path_scaling, robot_origin.y - pose_sequence[i].position.x*path_scaling);
        end = cv::Point(robot_origin.x - pose_sequence[i+1].position.y*path_scaling, robot_origin.y - pose_sequence[i+1].position.x*path_scaling);

        if (start.inside(rect) && end.inside(rect)){
            if (flg_showLines == true){
                line( cv_img, start, end, color, thickness, lineType );
            }
            if (flg_showVertices == true){
                circle( cv_img, end, 2, color, thickness, lineType);
            }
        }
    }

}

void asrl::safetyMonitor::user_interface::plot_pose_3sig_boundary(cv::Mat & cv_img,
                                                   const cv::Point & robot_origin,
                                                   const float & path_scaling,
                                                   const std::vector<geometry_msgs::Pose> & wc1_sequence,
                                                   const std::vector<geometry_msgs::Pose> & wc2_sequence,
                                                   const cv::Scalar & color)
{
    if (wc1_sequence.size() < 1 || wc2_sequence.size() < 1){
        return;
    } else if (wc1_sequence.size() != wc2_sequence.size()){
	ROS_INFO_STREAM(wc1_sequence.size() << ", " << wc2_sequence.size());
        ROS_WARN("wc1_sequence or wc2_sequence size not appropriate.");
        return;
    }

    int num_poses = wc1_sequence.size();

    cv::Point sig_outline[1][2*num_poses];
    for (int i = 0; i < num_poses; i++){
        sig_outline[0][i]                 = cv::Point(robot_origin.x - wc1_sequence[i].position.y*path_scaling, robot_origin.y - wc1_sequence[i].position.x*path_scaling);
        sig_outline[0][2*num_poses-1 - i] = cv::Point(robot_origin.x - wc2_sequence[i].position.y*path_scaling, robot_origin.y - wc2_sequence[i].position.x*path_scaling);
    }

    const cv::Point* ppt[1] = { sig_outline[0] };
    int npt[] = { 2*num_poses };
    cv::fillPoly(cv_img, ppt, npt, 1, color, 8);

}

void asrl::safetyMonitor::user_interface::plot_debug_box(cv::Mat & cv_img,
                        const cv::Point & box_tl,
                        const cv::Point & box_br,
                        std::string & signal_monitor_name,
                        int & signal_monitor_action)
{
    cv::Scalar red_color(234, 50, 50);
    cv::Scalar orng_color(206, 133, 0);
    cv::Scalar grn_color(31, 157, 0);
    cv::Scalar gry_color(156, 157, 157);

    cv::Scalar color;
    switch(signal_monitor_action){
        case PAUSE:
            color = red_color;
            break;
        case PAUSE_AND_RELOCALIZE:
            color = red_color;
            break;
        case SLOW:
            color = orng_color;
            break;
        case CONTINUE:
            color = grn_color;
            break;
        case UNKNOWN:
            color = gry_color;
            break;
        default:
            color = gry_color;
    }

    rectangle(cv_img, box_tl, box_br, color, -1, 8);

    int num_char = signal_monitor_name.length();
    int scaling_px_per_char = 10;
    int text_y_px = box_tl.y + (box_br.y-box_tl.y)/2 + scaling_px_per_char/2;
    int text_x_px = box_tl.x + (box_br.x-box_tl.x)/2 - num_char/2*scaling_px_per_char;
    cv::Point at(text_x_px, text_y_px);
    cv::Scalar textColor(0,0,0);

    cv::putText(cv_img, signal_monitor_name, at, 3, 0.5, textColor, 1, 8, false);


}

void asrl::safetyMonitor::user_interface::write_speed(cv::Mat & cv_img, const cv::Point & text_origin, std::string & vehicle_speed){

    cv::Scalar textColor(0,0,0);
    float font_scale = 0.75;
    cv::putText(cv_img, vehicle_speed, text_origin, 3, font_scale, textColor, 2, 8, false);

}


// The main function!
int main(int argc, char** argv)
{
    // Initialize Safety Monitor Node
    std::string nodeName = "Safety_Monitor_User_Interface";
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nodeHandle_("~");
    asrl::safetyMonitor::user_interface user_interface(nodeHandle_);

    ROS_INFO_STREAM("Booting up.");

    user_interface.spin();
}












