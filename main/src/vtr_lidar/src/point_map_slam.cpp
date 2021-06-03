#include "vtr_lidar/point_map_slam.h"

//-----------------------------------------------------------------------------------------------------------------------------
// Utilities
// *********

Plane3D extract_ground(vector<PointXYZ>& points,
					   vector<PointXYZ>& normals,
					   float angle_vertical_thresh,
					   float dist_thresh,
					   int max_iter,
					   bool mode_2D)
{

	// In case of 2D mode, take the minimum height and use vertical normal
	if (mode_2D)
	{
		PointXYZ A = points[0];
		for (auto& p : points)
		{
			if (p.z < A.z)
				A = p;
		}
		return Plane3D(A, PointXYZ(0, 0, 1));
	}

	// Get the points with vertical normal (angle_vertical_thresh should be in radians)
	vector<PointXYZ> valid_points;
	valid_points.reserve(points.size());
	size_t i = 0;
	float cos_thresh = cos(angle_vertical_thresh);
	for (auto& n : normals)
	{
		if (abs(n.z) > cos_thresh)
		{
			valid_points.push_back(points[i]);
		}
		i++;
	}

	// Random generator
	default_random_engine generator;
	uniform_int_distribution<size_t> distribution(0, valid_points.size() - 1);

	// RANSAC loop
	int best_votes = 0;
	Plane3D best_P;
	for (int i = 0; i < max_iter; i++)
	{
		// Draw 3 random points
		unordered_set<size_t> unique_inds;
		vector<PointXYZ> ABC;
		while (unique_inds.size() < 3)
		{
			size_t ind = distribution(generator);
			unique_inds.insert(ind);
			if (unique_inds.size() > ABC.size())
				ABC.push_back(valid_points[ind]);
		}

		// Get the corresponding plane
		Plane3D candidate_P(ABC[0], ABC[1], ABC[2]);

		// Avoid ill defined planes
		if (candidate_P.u.sq_norm() < 1e-5)
		{
			i--;
			continue;
		}

		// Get the number of votes for this plane
		int votes = candidate_P.in_range(valid_points, dist_thresh);

		// Save best plane
		if (votes > best_votes)
		{
			best_votes = votes;
			best_P = candidate_P;
		}
	}
	return best_P;
}
#if false
Eigen::Matrix4d transformListenerToEigenMatrix(const tf::TransformListener& listener, const string& target, const string& source, const ros::Time& stamp)
{
	tf::StampedTransform stampedTr;
	if (!listener.waitForTransform(target, source, stamp, ros::Duration(0.1)))
	{
		ROS_WARN_STREAM("Cannot get transformation from " << source << " to " << target);
		return Eigen::Matrix4d::Zero(4, 4);
	}
	else
	{
		listener.lookupTransform(target, source, stamp, stampedTr);
		Eigen::Affine3d eigenTr;
		tf::transformTFToEigen(stampedTr, eigenTr);
		return eigenTr.matrix();
	}
}

Eigen::Matrix4d odomMsgToEigenMatrix(const nav_msgs::Odometry& odom)
{
	Eigen::Affine3d eigenTr;
	tf::poseMsgToEigen(odom.pose.pose, eigenTr);
	return eigenTr.matrix();
}

nav_msgs::Odometry eigenMatrixToOdomMsg(const Eigen::Matrix4d& inTr, const string& frame_id, const ros::Time& stamp)
{
	nav_msgs::Odometry odom;
	odom.header.stamp = stamp;
	odom.header.frame_id = frame_id;

	// Fill pose
	const Eigen::Affine3d eigenTr(inTr);
	tf::poseEigenToMsg(eigenTr, odom.pose.pose);

	// Fill velocity, TODO: find proper computation from delta poses to twist
	//odom.child_frame_id = cloudMsgIn.header.frame_id;
	// odom.twist.covariance[0+0*6] = -1;
	// odom.twist.covariance[1+1*6] = -1;
	// odom.twist.covariance[2+2*6] = -1;
	// odom.twist.covariance[3+3*6] = -1;
	// odom.twist.covariance[4+4*6] = -1;
	// odom.twist.covariance[5+5*6] = -1;

	return odom;
}

tf::Transform eigenMatrixToTransform(const Eigen::Matrix4d& in_H)
{
	tf::Transform tfTr;
	const Eigen::Affine3d eigenTr(in_H);
	tf::transformEigenToTF(eigenTr, tfTr);
	return tfTr;
}
#endif

void PointMapSLAM::publish_2D_map()
{
	// Init meta-data
	OccupancyGridMsg map_message;
	map_message.info.width = map2D.maxPix.x - map2D.minPix.x + 1;
	map_message.info.height = map2D.maxPix.y - map2D.minPix.y + 1;
	map_message.info.origin.position.x = map2D.minPix.x * map2D.dl;
	map_message.info.origin.position.y = map2D.minPix.y * map2D.dl;
	map_message.info.origin.position.z = 0;

	map_message.info.origin.orientation.x = 0.0;
	map_message.info.origin.orientation.y = 0.0;
	map_message.info.origin.orientation.z = 0.0;
	map_message.info.origin.orientation.w = 1.0;
	map_message.info.resolution = map2D.dl;

	// Fill the ROS map object
	map_message.data = vector<int8_t>(map_message.info.width * map_message.info.height, 0);

	// Only consider point with height between 30cm and 1m30cm to avoid ground and still pass through doors
	for (auto& pix : map2D.samples)
	{
		size_t mapIdx = (size_t)((pix.first.x - map2D.minPix.x) + map_message.info.width * (pix.first.y - map2D.minPix.y));
		if (map2D.scores[pix.second] > 0.4)
			map_message.data[mapIdx] = 100;
	}

	//make sure to set the header information on the map
	map_message.header.stamp = node_->now();
	map_message.header.frame_id = params.map_frame;

	// Publish map and map metadata
	sst->publish(map_message);
	sstm->publish(map_message.info);

}

void PointMapSLAM::publish_map()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (auto pt : map.cloud.pts) 
    cloud.points.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));

  auto pc2_msg = std::make_shared<PointCloudMsg>();
  pcl::toROSMsg(cloud, *pc2_msg);
  pc2_msg->header.frame_id = "map";
  pc2_msg->header.stamp = stamp_;
  
  map_pub->publish(*pc2_msg);
}

void PointMapSLAM::publish_points(const rclcpp::Time& stamp, const vector<PointXYZ>& pts)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (auto pt : pts) 
    cloud.points.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));

  auto pc2_msg = std::make_shared<PointCloudMsg>();
  pcl::toROSMsg(cloud, *pc2_msg);
  pc2_msg->header.frame_id = "cam0";
  pc2_msg->header.stamp = stamp;

  pcd_pub->publish(*pc2_msg);
}

std::ostream &operator<<(std::ostream &os, const SLAM_params &s) {
  os << "SLAM Parameters" << endl
     << "  map_voxel_size:" << s.map_voxel_size << endl
     << "  frame_voxel_size:" << s.frame_voxel_size << endl
     << "  map2d_pixel_size:" << s.map2d_pixel_size << endl
     << "  map2d_max_count:" << s.map2d_max_count << endl
     << "  map2d_zMin:" << s.map2d_zMin << endl
     << "  map2d_zMax:" << s.map2d_zMax << endl
     << "  lidar_n_lines:" << s.lidar_n_lines << endl
     << "  motion_distortion:" << s.motion_distortion << endl
     << "  h_scale:" << s.h_scale << endl
     << "  r_scale:" << s.r_scale << endl
     << "  outl_rjct_passes:" << s.outl_rjct_passes << endl
     << "  outl_rjct_thresh:" << s.outl_rjct_thresh << endl
     << "  filtering:" << s.filtering << endl
     << "  gt_filter:" << s.gt_filter << endl
     << s.icp_params;
  return os;
}

//-----------------------------------------------------------------------------------------------------------------------------
// SLAM function
// *************

void PointMapSLAM::gotCloud(const PointCloudMsg::SharedPtr msg) {

  std::cout << "got cloud!" << std::endl;

  //////////////////////
  // Optional verbose //
  //////////////////////

  vector<string> clock_str;
  vector<clock_t> t;
  if (params.verbose) {
    clock_str.reserve(20);
    t.reserve(20);
    clock_str.push_back("Msg filtering ..... ");
    clock_str.push_back("tf listener ....... ");
    clock_str.push_back("Polar convertion .. ");
    clock_str.push_back("Outlier reject .... ");
    clock_str.push_back("Grid subsampling .. ");
    clock_str.push_back("Frame normals ..... ");
    clock_str.push_back("Normals filter .... ");
    clock_str.push_back("ICP localization .. ");
    clock_str.push_back("Publish tf ........ ");
    clock_str.push_back("Align frame ....... ");
    clock_str.push_back("Map update ........ ");
    clock_str.push_back("Map publish ..... ");
    clock_str.push_back("Ground extract .... ");
    clock_str.push_back("Map2D update ...... ");
    clock_str.push_back("Map2D publish ..... ");
  }
  t.push_back(std::clock());

  //////////////////////////////
  // Read point cloud message //
  //////////////////////////////

  // Get the number of points
  size_t N = (size_t)(msg->width * msg->height);

  // Get timestamp
  rclcpp::Time stamp{msg->header.stamp};
  stamp_ = stamp;

#if false
	// Ignore frames if not enough points
	if (N < 100)
	{
		ROS_WARN_STREAM("Frame #" << msg->header.seq << " with only " << N << " points is ignored.");
		return;
	}
#endif

	// Loop over points and copy in vector container. Do the filtering if necessary
	vector<PointXYZ> f_pts;
	f_pts.reserve(N);
#if false
	if (params.filtering)
	{
		sensor_msgs::PointCloud2ConstIterator<float> iter_i(*msg, "intensity"), iter_x(*msg, "x"), iter_y(*msg, "y");
		for (sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
			 iter_z != iter_z.end();
			 ++iter_x, ++iter_y, ++iter_z, ++iter_i)
		{
			// Reject points with wrong labels
			if (find(params.loc_labels.begin(), params.loc_labels.end(), (int)*iter_i) == params.loc_labels.end())
				continue;

			// Reject NaN values
			if (isnan(*iter_x) || isnan(*iter_y) || isnan(*iter_z))
			{
				ROS_WARN_STREAM("rejected for NaN in point(" << *iter_x << ", " << *iter_y << ", " << *iter_z << ")");
				continue;
			}

			// Add kept points to the vector container
			f_pts.push_back(PointXYZ(*iter_x, *iter_y, *iter_z));
		}
	}
	else
	{
#endif
		for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");
			 iter_x != iter_x.end();
			 ++iter_x, ++iter_y, ++iter_z)
		{
			// Add all points to the vector container
			f_pts.push_back(PointXYZ(*iter_x, *iter_y, *iter_z));
		}
#if false
	}
#endif
	t.push_back(std::clock());

	///////////////////////////////////////////
	// Get init matrix from current odometry //
	///////////////////////////////////////////
#if false
	// Get current pose of the scanner in the odom frame
	Eigen::Matrix4d H_OdomToScanner;
	try
	{
		H_OdomToScanner = transformListenerToEigenMatrix(tfListener, msg->header.frame_id, params.odom_frame, stamp);
	}
	catch (tf::ExtrapolationException e)
	{
		ROS_ERROR_STREAM("Extrapolation Exception. stamp = " << stamp << " now = " << ros::Time::now() << " delta = " << ros::Time::now() - stamp);
		return;
	}

	// If no odometry is given, previous one
	if (H_OdomToScanner.lpNorm<1>() < 0.001)
		H_OdomToScanner = last_H.inverse() * H_OdomToMap;

	// Get the pose of the scanner in the map
	Eigen::Matrix4d H_scannerToMap_init = H_OdomToMap * H_OdomToScanner.inverse();
#else
  // Yuchen temporarily set everything to identity
  Eigen::Matrix4d H_OdomToScanner = Eigen::Matrix4d::Identity(4, 4);
	// Get the pose of the scanner in the map
	Eigen::Matrix4d H_scannerToMap_init = H_OdomToMap * H_OdomToScanner.inverse();
#endif
	t.push_back(std::clock());

	// ROS_WARN_STREAM("TOdomToScanner(" << params.odom_frame << " to " << msg->header.frame_id << "):\n" << H_OdomToScanner);
	// ROS_WARN_STREAM("TOdomToMap(" << params.odom_frame << " to " << params.map_frame << "):\n" << H_OdomToMap);
	// ROS_WARN_STREAM("TscannerToMap (" << msg->header.frame_id << " to " << params.map_frame << "):\n" << H_scannerToMap_init);

	//////////////////////////////////////////
	// Preprocess frame and compute normals //
	//////////////////////////////////////////

	// Create a copy of points in polar coordinates
	vector<PointXYZ> polar_pts(f_pts);
	cart2pol_(polar_pts);

	t.push_back(std::clock());

	// Get lidar angle resolution
	float minTheta, maxTheta;
	float lidar_angle_res = get_lidar_angle_res(polar_pts, minTheta, maxTheta, params.lidar_n_lines);

	// Define the polar neighbors radius in the scaled polar coordinates
	float polar_r = 1.5 * lidar_angle_res;

	// Apply log scale to radius coordinate (in place)
	lidar_log_radius(polar_pts, polar_r, params.r_scale);

#if false
	// Remove outliers (only for real frames)
	if (params.motion_distortion)
	{
		// Get an outlier score
		vector<float> scores(polar_pts.size(), 0.0);
		detect_outliers(polar_pts, scores, params.lidar_n_lines, lidar_angle_res, minTheta, params.outl_rjct_passes, params.outl_rjct_thresh);

		// Remove points with negative score
		filter_pointcloud(f_pts, scores, 0);
		filter_pointcloud(polar_pts, scores, 0);
	}
#endif

	t.push_back(std::clock());

	// Get subsampling of the frame in carthesian coordinates
	vector<PointXYZ> sub_pts;
	vector<size_t> sub_inds;
	grid_subsampling_centers(f_pts, sub_pts, sub_inds, params.frame_voxel_size);

  publish_points(stamp, sub_pts);

	t.push_back(std::clock());

	// Convert sub_pts to polar and rescale
	vector<PointXYZ> polar_queries0(sub_pts);
	cart2pol_(polar_queries0);
	vector<PointXYZ> polar_queries(polar_queries0);
	lidar_log_radius(polar_queries, polar_r, params.r_scale);
	lidar_horizontal_scale(polar_queries, params.h_scale);

	// ROS_WARN_STREAM(" ------> " << f_pts.size() << " " << sub_pts.size() << " " << sub_inds.size() << " " << params.frame_voxel_size);

	/////////////////////
	// Compute normals //
	/////////////////////

	// Init result containers
	vector<PointXYZ> normals;
	vector<float> norm_scores;

	// Apply horizontal scaling (to have smaller neighborhoods in horizontal direction)
	lidar_horizontal_scale(polar_pts, params.h_scale);

	// Call polar processing function
	extract_lidar_frame_normals(f_pts, polar_pts, sub_pts, polar_queries, normals, norm_scores, polar_r);

	t.push_back(std::clock());

	// Better normal score based on distance and incidence angle
	vector<float> icp_scores(norm_scores);
	smart_icp_score(polar_queries0, icp_scores);
	smart_normal_score(sub_pts, polar_queries0, normals, norm_scores);

	// Remove points with a low score
	float min_score = 0.01;
	filter_pointcloud(sub_pts, norm_scores, min_score);
	filter_pointcloud(normals, norm_scores, min_score);
	filter_floatvector(icp_scores, norm_scores, min_score);
	filter_floatvector(norm_scores, min_score);

	t.push_back(std::clock());

	/////////////////////////////////
	// Align frame on map with ICP //
	/////////////////////////////////

	// Create result containers
	ICP_results icp_results;

	// If no map is available, use init_H as first pose
	if (map.size() < 1)
	{
		icp_results.transform = H_scannerToMap_init;
	}
	else
	{
		params.icp_params.init_transform = H_scannerToMap_init;
		PointToMapICP(sub_pts, icp_scores, map, params.icp_params, icp_results);
	}

	t.push_back(std::clock());

	///////////////////////
	// Publish transform //
	///////////////////////

	// Compute tf
	//publishStamp = stamp;
	//publishLock.lock();
	H_OdomToMap = icp_results.transform * H_OdomToScanner;
#if false
	// Publish tf
	tfBroadcaster.sendTransform(tf::StampedTransform(eigenMatrixToTransform(H_OdomToMap), stamp, params.map_frame, params.odom_frame));
	// ROS_WARN_STREAM("TOdomToMap:\n" << H_OdomToMap);
#endif
	t.push_back(std::clock());

	////////////////////
	// Update the map //
	////////////////////

	if (params.motion_distortion)
	{
		throw std::invalid_argument("motion_distortion not handled yet");
		// TODO Here:	- Handle case of motion distorsion
		//				- optimize by using the phis computed in ICP
	}
	else
	{
		Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat((float *)sub_pts.data(), 3, sub_pts.size());
		Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> norms_mat((float *)normals.data(), 3, normals.size());
		Eigen::Matrix3f R_tot = (icp_results.transform.block(0, 0, 3, 3)).cast<float>();
		Eigen::Vector3f T_tot = (icp_results.transform.block(0, 3, 3, 1)).cast<float>();
		pts_mat = (R_tot * pts_mat).colwise() + T_tot;
		norms_mat = R_tot * norms_mat;
	}

	t.push_back(std::clock());

	// The update function is called only on subsampled points as the others have no normal
	map.update(sub_pts, normals, norm_scores);

	t.push_back(std::clock());

  // Publish 3D map point cloud

  publish_map();
  t.push_back(std::clock());

	// Detect ground plane for height filtering
	Plane3D ground_P = extract_ground(sub_pts, normals);

	t.push_back(std::clock());

	// Update the 2D map
	PointXYZ center;
	if (params.motion_distortion)
	{
		throw std::invalid_argument("motion_distortion not handled yet");
		// TODO: handle this case
	}
	else
	{
		Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat((float *)f_pts.data(), 3, f_pts.size());
		Eigen::Matrix3f R_tot = (icp_results.transform.block(0, 0, 3, 3)).cast<float>();
		Eigen::Vector3f T_tot = (icp_results.transform.block(0, 3, 3, 1)).cast<float>();
		pts_mat = (R_tot * pts_mat).colwise() + T_tot;
		center.x = T_tot.x();
		center.y = T_tot.y();
		center.z = T_tot.z();
	}

	// DEBUG ////////////////////////////

	// if (map2D.size() > 0 && n_frames % 1 == 0)
	// {
	// 	string path = "/home/hth/Myhal_Simulation/simulated_runs/";
	// 	char buffer[200];
	// 	sprintf(buffer, "debug_frame_%05d.ply", n_frames);
	// 	string filepath = path + string(buffer);
	// 	vector<float> distances;
	// 	ground_P.point_distances(f_pts, distances);
	// 	save_cloud(filepath, f_pts, distances);

	// 	map.debug_save_ply(path, n_frames);
	// 	map2D.debug_save_ply(path, n_frames);

	// 	ROS_WARN_STREAM(">>>>>>>>>>>>> " << n_frames << ": " << ground_P.u << " " << ground_P.d);
	// }

	/////////////////////////////////////

	map2D.update_from_3D(f_pts, center, ground_P, params.map2d_zMin, params.map2d_zMax);

	t.push_back(std::clock());

	publish_2D_map();

	t.push_back(std::clock());

	// Update the last pose for future frames
	last_H = icp_results.transform;

	// Update number of frames
	n_frames++;

	// Save all poses
	all_H.push_back(icp_results.transform);
	f_times.push_back(stamp);

	////////////////////////
	// Debugging messages //
	////////////////////////

	if (params.verbose)
	{
		for (size_t i = 0; i < min(t.size() - 1, clock_str.size()); i++)
		{
			double duration = 1000 * (t[i + 1] - t[i]) / (double)CLOCKS_PER_SEC;
			cout << clock_str[i] << duration << " ms" << endl;
		}
		cout << endl
			 << "***********************" << endl
			 << endl;
	}

  std::cout << "cloud processed!" << std::endl;
  result_pub->publish(ResultMsg());
	return;

}

// void PointMapSLAM::update_transforms(const tf::tfMessage::ConstPtr& msg)
//{
//    // TODO
//
//    return;
//}