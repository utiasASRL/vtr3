std_msgs/Header header




# Path in robot frame - flattened Nx3 array
float32[] path

# array of size 5x2 for the MPC v,w solution
float32[] mpc_solution

# array of size 3x1 for robot x,y,theta pose in path frame
float32[3] robot_pose

float32[3] closest_interpolated_node_pose

# accessories that are useful for RL
float32 p_value
float32 distance_to_closest_interpolated_node
float32 theta_offset_to_closest_interpolated_node
float32 distance_to_goal_node
float32 theta_offset_to_goal_node