#include "vtr_path_planning/cbit/visualization_utils.hpp"

namespace vtr {
namespace path_planning {

// Default constructor definition
VisualizationUtils::VisualizationUtils() {
}

VisualizationUtils::VisualizationUtils(rclcpp::Node::SharedPtr node) {
    CLOG(ERROR, "path_planning.cbit") << "Trying to Make this shit";
    tf_bc_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    mpc_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("mpc_prediction", 10);
    robot_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("robot_path", 10);
    path_pub_ = node->create_publisher<nav_msgs::msg::Path>("planning_path", 10);
    corridor_pub_l_ = node->create_publisher<nav_msgs::msg::Path>("corridor_path_left", 10);
    corridor_pub_r_ = node->create_publisher<nav_msgs::msg::Path>("corridor_path_right", 10);
    ref_pose_pub_tracking_ = node->create_publisher<geometry_msgs::msg::PoseArray>("mpc_ref_pose_array_tracking", 10);
    ref_pose_pub_homotopy_ = node->create_publisher<geometry_msgs::msg::PoseArray>("mpc_ref_pose_array_homotopy", 10);
    CLOG(ERROR, "path_planning.cbit") << "successfully made this shit";
}

void VisualizationUtils::visualize(
    const tactic::Timestamp& stamp,
    const tactic::EdgeTransform& T_w_p,
    const tactic::EdgeTransform& T_p_r,
    const tactic::EdgeTransform& T_p_r_extp,
    const tactic::EdgeTransform& T_p_r_extp_mpc,
    const std::vector<lgmath::se3::Transformation>& mpc_prediction,
    const std::vector<lgmath::se3::Transformation>& robot_prediction,
    const std::vector<lgmath::se3::Transformation>& tracking_pose_vec,
    const std::vector<lgmath::se3::Transformation>& homotopy_pose_vec,
    const std::shared_ptr<std::vector<Pose>> cbit_path_ptr,
    const std::shared_ptr<CBITCorridor> corridor_ptr)
    {
    /// Publish the current frame for planning
    {
        Eigen::Affine3d T(T_w_p.matrix());
        auto msg = tf2::eigenToTransform(T);
        msg.header.stamp = rclcpp::Time(stamp);
        msg.header.frame_id = "world";
        msg.child_frame_id = "planning frame";
        tf_bc_->sendTransform(msg);
    }

    /// Publish the current robot in the planning frame
    {
        Eigen::Affine3d T(T_p_r.matrix());
        auto msg = tf2::eigenToTransform(T);
        msg.header.frame_id = "planning frame";
        msg.header.stamp = rclcpp::Time(stamp);
        msg.child_frame_id = "robot planning";
        tf_bc_->sendTransform(msg);
    }

    // Publish robot pose extrapolated using odometry
    {
        Eigen::Affine3d T(T_p_r_extp.matrix());
        auto msg = tf2::eigenToTransform(T);
        msg.header.frame_id = "planning frame";
        msg.header.stamp = rclcpp::Time(stamp);
        msg.child_frame_id = "robot planning (extrapolated)";
        tf_bc_->sendTransform(msg);
    }

    // Publish MPC extrapolated current robot pose
    {
        Eigen::Affine3d T(T_p_r_extp_mpc.matrix());
        auto msg = tf2::eigenToTransform(T);
        msg.header.frame_id = "planning frame";
        msg.header.stamp = rclcpp::Time(stamp);
        msg.child_frame_id = "robot planning (extrapolated) mpc";
        tf_bc_->sendTransform(msg);
    }

    /// Publishing the MPC horizon prediction
    {
        nav_msgs::msg::Path mpc_path;
        mpc_path.header.frame_id = "world";
        mpc_path.header.stamp = rclcpp::Time(stamp);
        auto& poses = mpc_path.poses;

        // intermediate states
        for (unsigned i = 0; i < mpc_prediction.size(); ++i) {
        auto& pose = poses.emplace_back();
        pose.pose = tf2::toMsg(Eigen::Affine3d(mpc_prediction[i].matrix()));
        }
        mpc_path_pub_->publish(mpc_path);
    }

    /// Publishing the history of the robots actual pose
    {
        nav_msgs::msg::Path robot_path;
        robot_path.header.frame_id = "world";
        robot_path.header.stamp = rclcpp::Time(stamp);
        auto& poses = robot_path.poses;

        // intermediate states
        for (unsigned i = 0; i < robot_prediction.size(); ++i) {
        auto& pose = poses.emplace_back();
        pose.pose = tf2::toMsg(Eigen::Affine3d(robot_prediction[i].matrix()));
        }
        robot_path_pub_->publish(robot_path);
    }


    // Attempting to publish the actual path which we are receiving from the shared pointer in the cbitplanner
    // The path is stored as a vector of se3 Pose objects from cbit/utils, need to iterate through and construct proper ros2 nav_msgs PoseStamped

    {
        nav_msgs::msg::Path path;
        path.header.frame_id = "world";
        path.header.stamp = rclcpp::Time(stamp);
        auto& poses = path.poses;

        // iterate through the path
        //CLOG(INFO, "path_planning.cbit") << "Trying to publish the path, the size is: " << (*cbit_path_ptr).size();
        geometry_msgs::msg::Pose test_pose;
        for (unsigned i = 0; i < (*cbit_path_ptr).size(); ++i)
        {
        auto& pose = poses.emplace_back();
        //pose.pose = tf2::toMsg(Eigen::Affine3d(T_p_i_vec[i].matrix())); // Example for how to grab the transform from a transform with covariance data type
        test_pose.position.x = (*cbit_path_ptr)[i].x;
        test_pose.position.y = (*cbit_path_ptr)[i].y;
        test_pose.position.z = (*cbit_path_ptr)[i].z;
        test_pose.orientation.x = 0.0;
        test_pose.orientation.y = 0.0;
        test_pose.orientation.z = 0.0;
        test_pose.orientation.w = 1.0;
        pose.pose = test_pose;
        }

        path_pub_->publish(path);
    }



    // Attempting to Publish the left and right dynamic corridor for the current path homotopy class
    {
        nav_msgs::msg::Path corridor_left;
        corridor_left.header.frame_id = "world";
        corridor_left.header.stamp = rclcpp::Time(stamp);
        auto& poses_l = corridor_left.poses;

        nav_msgs::msg::Path corridor_right;
        corridor_right.header.frame_id = "world";
        corridor_right.header.stamp = rclcpp::Time(stamp);
        auto& poses_r = corridor_right.poses;

        // iterate through the corridor paths
        geometry_msgs::msg::Pose test_pose_l;
        geometry_msgs::msg::Pose test_pose_r;
        for (unsigned i = 0; i < corridor_ptr->x_left.size(); i++) 
        {
        //lhs
        auto& pose_l = poses_l.emplace_back();
        test_pose_l.position.x = corridor_ptr->x_left[i];
        test_pose_l.position.y = corridor_ptr->y_left[i];
        test_pose_l.position.z = 0.0; // setting this 0.0 for now for flat world assumption, but long term we might want to add a z component
        test_pose_l.orientation.x = 0.0;
        test_pose_l.orientation.y = 0.0;
        test_pose_l.orientation.z = 0.0;
        test_pose_l.orientation.w = 1.0;
        pose_l.pose = test_pose_l;

        // rhs
        auto& pose_r = poses_r.emplace_back();
        test_pose_r.position.x = corridor_ptr->x_right[i];
        test_pose_r.position.y = corridor_ptr->y_right[i];
        test_pose_r.position.z = 0.0; // setting this 0.0 for now for flat world assumption, but long term we might want to add a z component
        test_pose_r.orientation.x = 0.0;
        test_pose_r.orientation.y = 0.0;
        test_pose_r.orientation.z = 0.0;
        test_pose_r.orientation.w = 1.0;
        pose_r.pose = test_pose_r;
        }

        corridor_pub_l_->publish(corridor_left);
        corridor_pub_r_->publish(corridor_right);
    }

    // Attempting to Publish the reference poses used in the mpc optimization as a pose array
    {
        // create a PoseArray message
        geometry_msgs::msg::PoseArray pose_array_msg;
        pose_array_msg.header.frame_id = "world";

        // fill the PoseArray with some sample poses
        for (int i = 0; i < tracking_pose_vec.size(); i++) {
            geometry_msgs::msg::Pose pose;
            auto T1 = tracking_pose_vec[i].matrix();
            pose.position.x = T1(0,3);
            pose.position.y = T1(1,3);;
            pose.position.z = T1(2,3);;
            pose.orientation.w = 1.0;
            pose_array_msg.poses.push_back(pose);
        }
        ref_pose_pub_tracking_->publish(pose_array_msg);
    }

        // Attempting to Publish the reference poses used in the mpc optimization as a pose array
    {
        // create a PoseArray message
        geometry_msgs::msg::PoseArray pose_array_msg;
        pose_array_msg.header.frame_id = "world";

        // fill the PoseArray with some sample poses
        for (int i = 0; i < homotopy_pose_vec.size(); i++) {
            geometry_msgs::msg::Pose pose;
            auto T2 = homotopy_pose_vec[i].matrix();
            pose.position.x = T2(0,3);
            pose.position.y = T2(1,3);;
            pose.position.z = T2(2,3);;
            pose.orientation.w = 1.0;
            pose_array_msg.poses.push_back(pose);
        }
        ref_pose_pub_homotopy_->publish(pose_array_msg);
    }
    return;
    }
    


} // namespace path_planning
} // namespace vtr