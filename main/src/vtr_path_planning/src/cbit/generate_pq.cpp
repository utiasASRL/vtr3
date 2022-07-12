#include "vtr_path_planning/cbit/generate_pq.hpp"
#include "vtr_path_planning/cbit/config.hpp"
#include "vtr_path_planning/cbit/utils.hpp"

// Path Constructor:
Path::Path(Config config, std::vector<Pose> initial_path)
{   
    double alpha = config.alpha;

    // Process the path
    disc_path = initial_path;

    // TODO: Cubic spline the discrete path to make it have smooth derivatives
    // TEMP DEBUG, set the path equal to discrete path (no interpolations yet)
    path = disc_path;

    // Iterate through all poses in disc_path, assign a p coordinate value to the pose to use for the curvilinear space reference
    int vect_size = disc_path.size();
    p.reserve(vect_size);
    p.push_back(0);
    for (int i=1; i<vect_size; i++)
    {
        p.push_back(p[i-1] + delta_p_calc(disc_path[i-1], disc_path[i], alpha));
    }
    
}

// Calculating the distance between se(3) poses including a heading contribution
double Path::delta_p_calc(Pose start_pose, Pose end_pose, double alpha)
{
    double dx = start_pose.x - end_pose.x;
    double dy = start_pose.y - end_pose.y;
    double dz = start_pose.z - end_pose.z;

    // For the angular contribution, we need to be very careful about angle wrap around, cant simply just take the difference
    double dyaw = end_pose.yaw - start_pose.yaw;

    double yaw_arr[3] = {std::abs(dyaw), std::abs(dyaw + (2 * M_PI)), std::abs(dyaw - (2 * M_PI))};
    
    double abs_angle = 2.0 * M_PI;
    for (int i = 0; i < 3; i++)
    {
        if (yaw_arr[i] < abs_angle)
        {
            abs_angle = yaw_arr[i];
        }
    }

    return sqrt((dx * dx) + (dy * dy) + (dz * dz) + (alpha * abs_angle * abs_angle));
}

void Path::spline_curvature()
{
    // TODO
    auto test = 1;
    
}