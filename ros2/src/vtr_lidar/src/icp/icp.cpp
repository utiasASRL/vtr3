
#include "vtr_lidar/icp/icp.h"


std::ostream &operator<<(std::ostream &os, const ICP_params &s) {
  os << "ICP Parameters" << endl
     << "  n_samples:" << s.n_samples << endl
     << "  max_pairing_dist:" << s.max_pairing_dist << endl
     << "  max_planar_dist:" << s.max_planar_dist << endl
     << "  max_iter:" << s.max_iter << endl
     << "  avg_steps:" << s.avg_steps << endl
     << "  rotDiffThresh:" << s.rotDiffThresh << endl
     << "  transDiffThresh:" << s.transDiffThresh << endl
     << "  motion_distortion:" << s.motion_distortion << endl
     << "  init_phi:" << s.init_phi << endl
     << "  motion_distortion:" << s.motion_distortion << endl;
	return os;
}

// Utils
// *****

void regu_pose_cycle(vector<Eigen::Matrix4d>& H, vector<float>& H_w)
{
	// Bundle regularization
	//
	// H[i] = H(i,i-1) transformation that alignes frame i on frame i-1 (i is modulo B)
	// Example for B = 5
	// We Have H04 H10 H21 H32 H43
	// For H10 we compute H_bis = H40 * H34 * H23 * H12 which we want equal to H10
	// Therefore newH10 = pose_interp(w1, H10, H_bis, 0)
	// To regularize all poses together, all wi = 1/B. But if you favor one transformation (the largest for example). Change w

	// Ensure sum of w equals 1
	float reg_w = 1.0 / std::accumulate(H_w.begin(), H_w.end(), 0);
	for (auto& w : H_w)
		w *= reg_w;

	size_t B = H.size();
	for (size_t b = 0; b < B; b++)
	{
		Eigen::Matrix4d H_bis = Eigen::Matrix4d::Identity(4, 4);
		for (size_t bb = b + 1; bb < b + B; bb++)
		{
			size_t bb_0 = bb % B;
			H_bis = H[bb_0].inverse() * H_bis;
		}
		H[b] = pose_interp(1.0 / B, H[b], H_bis, 0);
	}
}


Eigen::Matrix4d pose_interp(float t, Eigen::Matrix4d const& H1, Eigen::Matrix4d const& H2, int verbose)
{
	// Assumes 0 < t < 1
	Eigen::Matrix3d R1 = H1.block(0, 0, 3, 3);
	Eigen::Matrix3d R2 = H2.block(0, 0, 3, 3);

	// Rotations to quaternions
	Eigen::Quaternion<double> rot1(R1);
	Eigen::Quaternion<double> rot2(R2);
	Eigen::Quaternion<double> rot3 = rot1.slerp(t, rot2);

	// ------------------ ICI ----------------------

	if (verbose > 0)
	{
		cout << R2.determinant() << endl;
		cout << R2 << endl;
		cout << "[" << rot1.x() << " " << rot1.y() << " " << rot1.z() << " " << rot1.w() << "] -> ";
		cout << "[" << rot2.x() << " " << rot2.y() << " " << rot2.z() << " " << rot2.w() << "] / " << t << endl;
		cout << "[" << rot3.x() << " " << rot3.y() << " " << rot3.z() << " " << rot3.w() << "]" << endl;
		cout << rot2.toRotationMatrix() << endl;
		cout << rot3.toRotationMatrix() << endl;
	}

	// Translations to vectors
	Eigen::Vector3d trans1 = H1.block(0, 3, 3, 1);
	Eigen::Vector3d trans2 = H2.block(0, 3, 3, 1);

	// Interpolation (not the real geodesic path, but good enough)
	Eigen::Affine3d result;
	result.translation() = (1.0 - t) * trans1 + t * trans2;
	result.linear() = rot1.slerp(t, rot2).normalized().toRotationMatrix();

	return result.matrix();
}



// Minimizer
// *********


void SolvePoint2PlaneLinearSystem(const Matrix6d& A, const Vector6d& b, Vector6d& x)
{
	// Define a slover on matrix A
	Eigen::FullPivHouseholderQR<Matrix6d> Aqr(A);

	if (!Aqr.isInvertible())
	{
		cout << "WARNING: Full minimization instead of cholesky" << endl;
		// Solve system (but solver is slow???)
		x = Aqr.solve(b);

		/*
		// Solve reduced problem R1 x = Q1^T b instead of QR x = b, where Q = [Q1 Q2] and R = [ R1 ; R2 ] such that ||R2|| is small (or zero) and therefore A = QR ~= Q1 * R1
		const int rank = Aqr.rank();
		const int rows = A.rows();
		const Matrix Q1t = Aqr.matrixQ().transpose().block(0, 0, rank, rows);
		const Matrix R1 = (Q1t * A * Aqr.colsPermutation()).block(0, 0, rank, rows);

		const bool findMinimalNormSolution = true; // TODO is that what we want?

		// The under-determined system R1 x = Q1^T b is made unique ..
		if (findMinimalNormSolution) {
			// by getting the solution of smallest norm (x = R1^T * (R1 * R1^T)^-1 Q1^T b.
			x = R1.template triangularView<Eigen::Upper>().transpose() * (R1 * R1.transpose()).llt().solve(Q1t * b);
		}
		else {
			// by solving the simplest problem that yields fewest nonzero components in x
			x.block(0, 0, rank, 1) = R1.block(0, 0, rank, rank).template triangularView<Eigen::Upper>().solve(Q1t * b);
			x.block(rank, 0, rows - rank, 1).setZero();
		}

		x = Aqr.colsPermutation() * x;

		BOOST_AUTO(ax, (A * x).eval());
		if (!b.isApprox(ax, 1e-5)) {
			LOG_INFO_STREAM("PointMatcher::icp - encountered almost singular matrix while minimizing point to plane distance. QR solution was too inaccurate. Trying more accurate approach using double precision SVD.");
			x = A.template cast<double>().jacobiSvd(ComputeThinU | ComputeThinV).solve(b.template cast<double>()).template cast<T>();
			ax = A * x;

			if ((b - ax).norm() > 1e-5 * std::max(A.norm() * x.norm(), b.norm())) {
				LOG_WARNING_STREAM("PointMatcher::icp - encountered numerically singular matrix while minimizing point to plane distance and the current workaround remained inaccurate."
					<< " b=" << b.transpose()
					<< " !~ A * x=" << (ax).transpose().eval()
					<< ": ||b- ax||=" << (b - ax).norm()
					<< ", ||b||=" << b.norm()
					<< ", ||ax||=" << ax.norm());
			}
		}
		*/
	}
	else
	{
		// Cholesky decomposition
		x = A.llt().solve(b);
	}
}


void PointToPlaneErrorMinimizer(vector<PointXYZ>& targets,
	vector<PointXYZ>& references,
	vector<PointXYZ>& refNormals,
	vector<float>& weights,
	vector<pair<size_t, size_t>>& sample_inds,
	Eigen::Matrix4d& mOut)
{
	// See: "Linear Least-Squares Optimization for Point-to-Plane ICP Surface Registration" (Kok-Lim Low)
	// init A and b matrice
	size_t N = sample_inds.size();
	Eigen::Matrix<double, Eigen::Dynamic, 6> A(N, 6);
	Eigen::Matrix<double, Eigen::Dynamic, 6> wA(N, 6);
	Eigen::Matrix<double, Eigen::Dynamic, 1> b(N, 1);

	// Fill matrices values
	bool tgt_weights = weights.size() == targets.size();
	bool ref_weights = weights.size() == references.size();
	int i = 0;
	for (const auto& ind : sample_inds)
	{
		// Target point
		double sx = (double)targets[ind.first].x;
		double sy = (double)targets[ind.first].y;
		double sz = (double)targets[ind.first].z;

		// Reference point
		double dx = (double)references[ind.second].x;
		double dy = (double)references[ind.second].y;
		double dz = (double)references[ind.second].z;

		// Reference point normal
		double nx = (double)refNormals[ind.second].x;
		double ny = (double)refNormals[ind.second].y;
		double nz = (double)refNormals[ind.second].z;

		// setup least squares system
		A(i, 0) = nz * sy - ny * sz;
		A(i, 1) = nx * sz - nz * sx;
		A(i, 2) = ny * sx - nx * sy;
		A(i, 3) = nx;
		A(i, 4) = ny;
		A(i, 5) = nz;
		b(i, 0) = nx * dx + ny * dy + nz * dz - nx * sx - ny * sy - nz * sz;

		// Apply weights if needed
		if (tgt_weights)
			wA.row(i) = A.row(i) * (double)weights[ind.first];
		else if (ref_weights)
			wA.row(i) = A.row(i) * (double)weights[ind.second];
		i++;
	}

	// linear least square matrices
	Matrix6d A_ = wA.transpose() * A;
	Vector6d b_ = wA.transpose() * b;

	// Solve linear optimization
	Vector6d x;
	SolvePoint2PlaneLinearSystem(A_, b_, x);

	// Get transformation rotation
	Eigen::Transform<double, 3, Eigen::Affine> transform;
	transform = Eigen::AngleAxis<double>(x.head(3).norm(), x.head(3).normalized());

	// Reverse roll-pitch-yaw conversion, very useful piece of knowledge, keep it with you all time!
	//const float pitch = -asin(transform(2,0));
	//const float roll = atan2(transform(2,1), transform(2,2));
	//const float yaw = atan2(transform(1,0) / cos(pitch), transform(0,0) / cos(pitch));
	//std::cerr << "d angles" << x(0) - roll << ", " << x(1) - pitch << "," << x(2) - yaw << std::endl;

	// Get transformation translation
	transform.translation() = x.segment(3, 3);

	// Convert to 4x4 matrix
	mOut = transform.matrix();

	if (mOut != mOut)
	{
		// Degenerate situation. This can happen when the source and reading clouds
		// are identical, and then b and x above are 0, and the rotation matrix cannot
		// be determined, it comes out full of NaNs. The correct rotation is the identity.
		mOut.block(0, 0, 3, 3) = Eigen::Matrix4d::Identity(3, 3);
	}
}



// ICP functions
// *************



void BundleICP(vector<PointXYZ>& points,
	vector<PointXYZ>& normals,
	vector<float>& weights,
	vector<int>& lengths,
	ICP_params& params,
	BundleIcpResults& results)
{
	// Parameters
	// **********

	size_t B = lengths.size();
	size_t N = points.size();
	size_t W = weights.size() / N;


	// Create KDTrees on the reference clouds
	// **************************************

	// Copy references (because the values in "points" are going to be modified)
	vector<PointXYZ> references(points);

	// PointCloud structure for Nanoflann KDTree (deep copy)
	vector<PointCloud> refClouds(B);
	size_t p0 = 0;
	for (size_t b = 0; b < B; b++)
	{
		refClouds[b].pts = vector<PointXYZ>(points.begin() + p0, points.begin() + p0 + lengths[b]);
		p0 += lengths[b];
	}

	// Tree parameters
	nanoflann::KDTreeSingleIndexAdaptorParams tree_params(10 /* max leaf */);

	// Vector of trees
	vector<PointXYZ_KDTree*> KDTrees;
	for (auto& cloud : refClouds)
		KDTrees.push_back(new PointXYZ_KDTree(3, cloud, tree_params));

	// Build KDTrees
	for (auto& index : KDTrees)
		index->buildIndex();

	// Create search parameters
	nanoflann::SearchParams search_params;
	//search_params.sorted = false;

	// Start ICP loop
	// **************

	// Matrix of original data (only shallow copy of ref clouds)
	vector<Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>>> refMats;
	for (auto& cloud : refClouds)
		refMats.push_back(Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>>((float*)cloud.pts.data(), 3, cloud.pts.size()));

	// Matrix for aligned data (Shallow copy of parts of the stacked "points" vector)
	p0 = 0;
	vector<Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>>> alignedMats;
	for (size_t b = 0; b < B; b++)
	{
		alignedMats.push_back(Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>>((float*)(points.data() + p0), 3, lengths[b]));
		p0 += lengths[b];
	}

	// Define sampling strategy
	vector<vector<size_t>> best_idx(B * W);
	default_random_engine generator;
	vector<discrete_distribution<int>> distributions;


	if (W == 1)
	{
		// Random generator
		p0 = 0;
		for (size_t b = 0; b < B; b++)
		{
			distributions.push_back(discrete_distribution<int>(weights.begin() + p0, weights.begin() + p0 + lengths[b]));
			p0 += lengths[b];
		}
	}
	else
	{
		// The query points will be the best scores Pick queries as the best scores
		p0 = 0;
		for (size_t b = 0; b < B; b++)
		{
			for (size_t w = 0; w < W; w++)
			{
				best_idx[b + w * B] = vector<size_t>(lengths[b]);
				for (size_t i = 0; i < (size_t)lengths[b]; i++)
					best_idx[b + w * B][i] = i + p0;

				stable_sort(best_idx[b + w * B].begin(), best_idx[b + w * B].end(),
					[&weights, &w, &W](size_t i1, size_t i2) {return weights[w + i1 * W] > weights[w + i2 * W]; });
			}
			p0 += lengths[b];
		}
	}

	// Convergence varaibles
	vector<float> mean_dT(B, 0);
	vector<float> mean_dR(B, 0);
	size_t max_it = params.max_iter;
	bool stop_cond = false;


	// Init result containers
	vector<Eigen::Matrix4d> H_icp(B);
	for (auto& frame_rms : results.all_rms)
		frame_rms.reserve(params.max_iter);

	vector<clock_t> t(6);
	vector<string> clock_str;
	clock_str.push_back("Random_Sample ... ");
	clock_str.push_back("KNN_search ...... ");
	clock_str.push_back("Optimization .... ");
	clock_str.push_back("Regularization .. ");
	clock_str.push_back("Result .......... ");

	for (size_t step = 0; step < max_it; step++)
	{
		/////////////////
		// Association //
		/////////////////

		t[0] = std::clock();

		// Only one weight means random sampling. We compute best indices at each iteration
		if (W == 1)
		{
			p0 = 0;
			for (size_t b = 0; b < B; b++)
			{
				if (params.n_samples < (size_t)lengths[b])
				{
					unordered_set<size_t> unique_inds;
					while (unique_inds.size() < params.n_samples)
						unique_inds.insert((size_t)distributions[b](generator));

					best_idx[b] = vector<size_t>(params.n_samples);
					size_t i = 0;
					for (const auto& ind : unique_inds)
					{
						best_idx[b][i] = ind + p0;
						i++;
					}
				}
				else
				{
					best_idx[b] = vector<size_t>(lengths[b]);
					for (size_t i = 0; i < (size_t)lengths[b]; i++)
					{
						best_idx[b][i] = i + p0;
						i++;
					}
				}
				p0 += lengths[b];
			}
		}

		t[1] = std::clock();

		// Find neighbors in previous cloud (previous modulo B)
		vector<vector<pair<size_t, size_t>>> all_sample_inds(B);
		for (size_t b = 0; b < B; b++)
		{
			// Init neighbors container
			all_sample_inds[b].reserve(params.n_samples * W);

			// Tree we search in is the previous frame
			size_t tree_b = (b + B - 1) % B;
			int tree_p0 = 0;
			for (size_t i = 0; i < tree_b; i++)
				tree_p0 += lengths[i];

			// Find nearest neigbors
			float rms2 = 0;
			for (size_t w = 0; w < W; w++)
			{
				size_t best_sample = 0;
				size_t i = 0;
				while (best_sample < best_idx[b + w * B].size() && i < params.n_samples)
				{

					// Get next best idx
					size_t candidate_ind = best_idx[b + w * B][best_sample++];
					vector<size_t>   candidate_neighb(1);
					vector<float> candidate_dist(1);

					// Get its neighbor
					KDTrees[tree_b]->knnSearch((float*)&points[candidate_ind],
						1,
						&candidate_neighb[0],
						&candidate_dist[0]);

					// Verify it is in range
					// PointXYZ diff = (points[all_sample_inds[b][i].second] - points[all_sample_inds[b][i].first]);
					// diff.dot(normals[all_sample_inds[b][i].second])
					if (candidate_dist[0] < params.max_pairing_dist * params.max_pairing_dist)
					{
						i++;
						PointXYZ diff = (references[candidate_neighb[0] + tree_p0] - points[candidate_ind]);
						float planar_dist = abs(diff.dot(normals[candidate_neighb[0] + tree_p0]));
						rms2 += planar_dist;
						all_sample_inds[b].push_back(pair<size_t, size_t>(candidate_ind, candidate_neighb[0] + tree_p0));
					}
				}
			}

			// Compute RMS
			results.all_rms[b].push_back(sqrt(rms2 / (float)all_sample_inds[b].size()));
		}

		t[2] = std::clock();

		//////////////////
		// Optimization //
		//////////////////

		// Minimize error
		for (size_t b = 0; b < B; b++)
		{
			// Do not use weights in minimization, because we used them for random sampling
			vector<float> ones(weights.size(), 1.0);
			PointToPlaneErrorMinimizer(points, references, normals, ones, all_sample_inds[b], H_icp[b]);
		}

		t[3] = std::clock();

		// Update result transforamtions
		for (size_t b = 0; b < B; b++)
			results.transforms[b] = H_icp[b] * results.transforms[b];


		////////////////////
		// Regularization //
		////////////////////

		// Apply regularization (Several timnes as it is a approximate regu)
		vector<float> H_w(B, 1.0f);
		size_t num_regu = 1;
		if (max_it < params.max_iter)
		{
			H_w[0] = 0;
			num_regu = 3;
		}
		for (size_t i = 0; i < num_regu; i++)
			regu_pose_cycle(results.transforms, H_w);

		t[4] = std::clock();

		// Apply final regularised transformations
		for (size_t b = 0; b < B; b++)
		{
			Eigen::Matrix3f R_tot = (results.transforms[b].block(0, 0, 3, 3)).cast<float>();
			Eigen::Vector3f T_tot = (results.transforms[b].block(0, 3, 3, 1)).cast<float>();
			alignedMats[b] = (R_tot * refMats[b]).colwise() + T_tot;
		}

		// Update all result matrices
		if (step == 0)
		{
			Eigen::MatrixXd new_H(4, results.all_transforms.cols());
			for (size_t b = 0; b < B; b++)
				new_H.block(0, b * 4, 4, 4) = Eigen::MatrixXd(results.transforms[b]);
			results.all_transforms = new_H;
		}
		else
		{
			Eigen::MatrixXd new_H(4, results.all_transforms.cols());
			for (size_t b = 0; b < B; b++)
				new_H.block(0, b * 4, 4, 4) = Eigen::MatrixXd(results.transforms[b]);
			Eigen::MatrixXd temp(results.all_transforms.rows() + 4, results.all_transforms.cols());
			temp.topRows(results.all_transforms.rows()) = results.all_transforms;
			temp.bottomRows(new_H.rows()) = new_H;
			results.all_transforms = temp;
		}

		t[5] = std::clock();

		///////////////////////
		// Check convergence //
		///////////////////////

		// Update variations
		if (!stop_cond && step > 0)
		{
			float avg_tot = (float)params.avg_steps;
			if (step == 1)
				avg_tot = 1.0;

			if (step > 0)
				for (size_t b = 0; b < B; b++)
				{
					// Get last transformation variations
					Eigen::Matrix3d R2 = results.all_transforms.block(results.all_transforms.rows() - 4, b * 4, 3, 3);
					Eigen::Matrix3d R1 = results.all_transforms.block(results.all_transforms.rows() - 8, b * 4, 3, 3);
					Eigen::Vector3d T2 = results.all_transforms.block(results.all_transforms.rows() - 4, b * 4 + 3, 3, 1);
					Eigen::Vector3d T1 = results.all_transforms.block(results.all_transforms.rows() - 8, b * 4 + 3, 3, 1);
					R1 = R2 * R1.transpose();
					T1 = -R1 * T1 + T2;
					float dT_b = T1.norm();
					float dR_b = acos((R1.trace() - 1) / 2);
					mean_dT[b] += (dT_b - mean_dT[b]) / avg_tot;
					mean_dR[b] += (dR_b - mean_dR[b]) / avg_tot;
				}
		}

		// Stop condition
		if (!stop_cond && step > params.avg_steps)
		{
			stop_cond = true;
			for (size_t b = 0; b < B; b++)
			{
				stop_cond = stop_cond && (mean_dT[b] < params.transDiffThresh);
				stop_cond = stop_cond && (mean_dR[b] < params.rotDiffThresh);
			}
			if (stop_cond)
			{
				// Do not stop right away. Have a last few averaging steps
				max_it = step + params.avg_steps;
			}
		}

		// Last call, average the last transformations
		if (step > max_it - 2)
		{
			for (size_t b = 0; b < B; b++)
			{
				Eigen::Matrix4d mH = Eigen::Matrix4d::Identity(4, 4);
				for (size_t s = 0; s < params.avg_steps; s++)
				{
					Eigen::Matrix4d H = results.all_transforms.block(results.all_transforms.rows() - 4 * (1 + s), b * 4, 4, 4);
					mH = pose_interp(1.0 / (float)(1 + s), mH, H, 0);
				}
				results.transforms[b] = mH;
				results.all_transforms.block(results.all_transforms.rows() - 4, b * 4, 4, 4) = mH;
			}
		}




		///////////// DEBUG /////////////

		//cout << "***********************" << endl;
		//for (size_t i = 0; i < t.size() - 1; i++)
		//{
		//	double duration = 1000 * (t[i + 1] - t[i]) / (double)CLOCKS_PER_SEC;
		//	cout << clock_str[i] << duration << " ms" << endl;
		//}
		//cout << " - - - - - - - - - - - - - ";
		//for (size_t i = 0; i < t.size() - 1; i++)
		//{
		//	double duration = 1000 * (t[i + 1] - t[i]) / (double)CLOCKS_PER_SEC;
		//	cout << duration << " ";
		//}
		//cout << endl;
		//cout << "***********************" << endl;

		//Eigen::Matrix3f R = H.block(0, 0, 3, 3);
		//Eigen::Vector3f T = H.block(0, 3, 3, 1);
		//cout << "dT = " << endl << T << endl;
		//cout << "dR = " << endl << T << endl;


		//if (step % 30 == 0)
		//{

		//	// Get bundle_inds
		//	vector<float> bundle_i(points.size(), 0.0);
		//	p0 = 0;
		//	for (size_t b = 0; b < B; b++)
		//	{
		//		for (int i = 0; i < (size_t) lengths[b]; i++)
		//			bundle_i[p0 + i] = (float)b;
		//		p0 += lengths[b];
		//	}

		//	// get picks
		//	vector<float> pickss(points.size(), 0.0);
		//	for (size_t b = 0; b < B; b++)
		//	{
		//		for (auto& inds : all_sample_inds[b])
		//			pickss[inds.first] = 1.0;
		//	}
		//	bundle_i.insert(bundle_i.end(), pickss.begin(), pickss.end());


		//	char buffer[100];
		//	sprintf(buffer, "cc_aligned_%03d.ply", (int)step);
		//	save_cloud(string(buffer), points, bundle_i);
		//}


		///////////// DEBUG /////////////

		//
		//	TODO: convergence check with RMSdiff? R_diff and T_diff?
		//	TODO: residual error on full cloud?
		//
	}


	// Final regularisation for good measure
	//for (size_t b = 0; b < B; b++)
	//{
	//	Eigen::Matrix4d dH = Eigen::Matrix4d::Identity(4, 4);
	//	for (size_t bb = b; bb < b + B; bb++)
	//	{
	//		size_t bb_0 = bb % B;
	//		dH = dH * results.transforms[bb_0];
	//		cout << " " << bb_0;
	//	}

	//	cout << endl << "dH" << b << " = " << endl;
	//	cout << dH << endl;
	//}

	//vector<float> H_w(B, 1.0f);
	//if (true)
	//	H_w[0] = 0;
	//average_poses(results.transforms, H_w);

	//for (size_t b = 0; b < B; b++)
	//{
	//	Eigen::Matrix4d dH = Eigen::Matrix4d::Identity(4, 4);
	//	for (size_t bb = b; bb < b + B; bb++)
	//	{
	//		size_t bb_0 = bb % B;
	//		dH = dH * results.transforms[bb_0];
	//		cout << " " << bb_0;
	//	}

	//	cout << endl << "dH" << b << " = " << endl;
	//	cout << dH << endl;
	//}





}


void PointToMapICPDebug(vector<PointXYZ>& tgt_pts,
	vector<float>& tgt_w,
	vector<PointXYZ>& map_points,
	vector<PointXYZ>& map_normals,
	vector<float>& map_scores,
	ICP_params& params,
	ICP_results& results)
{
	// Parameters
	// **********

	size_t N = tgt_pts.size();

	// Get angles phi of each points for motion distorsion
	vector<float> phis;
	float phi1 = 0;
	if (params.motion_distortion)
	{
		phis.reserve(tgt_pts.size());
		for (auto& p : tgt_pts)
		{
			phis.push_back(fmod(3 * M_PI / 2 - atan2(p.y, p.x), 2 * M_PI));
			if (phis.back() > phi1)
				phi1 = phis.back();
		}
	}

	//// Debug phi angles
	//for (auto& phi : phis)
	//{
	//	float t = (phi - params.init_phi) / (phi1 - params.init_phi);
	//	cout << t << " " << phi << endl;
	//}
	//cout << params.init_phi << " << " << phi1 << endl;

	//save_cloud("test_cpp_phi.ply", tgt_pts, phis);
	//cout << "          C++    " << params.init_phi << " " << phi1 << endl;


	// Create KDTrees on the reference clouds
	// **************************************

	// PointCloud structure for Nanoflann KDTree (deep copy)
	PointCloud map_cloud;
	map_cloud.pts = map_points;

	// Tree parameters
	nanoflann::KDTreeSingleIndexAdaptorParams tree_params(10 /* max leaf */);

	// Vector of trees
	PointXYZ_KDTree map_tree(3, map_cloud, tree_params);
	map_tree.buildIndex();

	// Create search parameters
	nanoflann::SearchParams search_params;
	//search_params.sorted = false;

	// Start ICP loop
	// **************

	// Matrix of original data (only shallow copy of ref clouds)
	Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> map_mat((float*)map_points.data(), 3, map_points.size());

	// Aligned points (Deep copy of targets)
	vector<PointXYZ> aligned(tgt_pts);

	// Matrix for original/aligned data (Shallow copy of parts of the points vector)
	Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> targets_mat((float*)tgt_pts.data(), 3, N);
	Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> aligned_mat((float*)aligned.data(), 3, N);

	// Apply initial transformation
	Eigen::Matrix3f R_init = (params.init_transform.block(0, 0, 3, 3)).cast<float>();
	Eigen::Vector3f T_init = (params.init_transform.block(0, 3, 3, 1)).cast<float>();
	aligned_mat = (R_init * targets_mat).colwise() + T_init;
	results.transform = params.init_transform;

	// Random generator
	default_random_engine generator;
	discrete_distribution<int> distribution(tgt_w.begin(), tgt_w.end());

	// Init result containers
	Eigen::Matrix4d H_icp;
	results.all_rms.reserve(params.max_iter);
	results.all_plane_rms.reserve(params.max_iter);


	// Convergence varaibles
	float mean_dT = 0;
	float mean_dR = 0;
	size_t max_it = params.max_iter;
	bool stop_cond = false;

	vector<clock_t> t(6);
	vector<string> clock_str;
	clock_str.push_back("Random_Sample ... ");
	clock_str.push_back("KNN_search ...... ");
	clock_str.push_back("Optimization .... ");
	clock_str.push_back("Regularization .. ");
	clock_str.push_back("Result .......... ");

	for (size_t step = 0; step < max_it; step++)
	{
		/////////////////
		// Association //
		/////////////////

		// Pick random queries (use unordered set to ensure uniqueness)
		vector<pair<size_t, size_t>> sample_inds;
		if (params.n_samples < N)
		{
			unordered_set<size_t> unique_inds;
			while (unique_inds.size() < params.n_samples)
				unique_inds.insert((size_t)distribution(generator));

			sample_inds = vector<pair<size_t, size_t>>(params.n_samples);
			size_t i = 0;
			for (const auto& ind : unique_inds)
			{
				sample_inds[i].first = ind;
				i++;
			}
		}
		else
		{
			sample_inds = vector<pair<size_t, size_t>>(N);
			for (size_t i = 0; i < N; i++)
			{
				sample_inds[i].first = i;
				i++;
			}
		}

		t[1] = std::clock();

		// Init neighbors container
		vector<float> nn_dists(sample_inds.size());

		// Find nearest neigbors
		// #pragma omp parallel for shared(max_neighbs) schedule(dynamic, 10) num_threads(n_thread)
		for (size_t i = 0; i < sample_inds.size(); i++)
			map_tree.knnSearch((float*)&aligned[sample_inds[i].first], 1, &sample_inds[i].second, &nn_dists[i]);

		t[2] = std::clock();


		///////////////////////
		// Distances metrics //
		///////////////////////

		// Erase sample_inds if dists is too big
		vector<pair<size_t, size_t>> filtered_sample_inds;
		filtered_sample_inds.reserve(sample_inds.size());
		float rms2 = 0;
		float prms2 = 0;
		for (size_t i = 0; i < sample_inds.size(); i++)
		{
			if (nn_dists[i] < params.max_pairing_dist)
			{
				// Keep samples
				filtered_sample_inds.push_back(sample_inds[i]);

				// Update pt2pt rms
				rms2 += nn_dists[i];

				// update pt2pl rms
				PointXYZ diff = (map_points[sample_inds[i].second] - aligned[sample_inds[i].first]);
				float planar_dist = abs(diff.dot(map_normals[sample_inds[i].second]));
				prms2 += planar_dist;
			}
		}
		// Compute RMS
		results.all_rms.push_back(sqrt(rms2 / (float)filtered_sample_inds.size()));
		results.all_plane_rms.push_back(sqrt(prms2 / (float)filtered_sample_inds.size()));

		t[3] = std::clock();


		//////////////////
		// Optimization //
		//////////////////

		// Minimize error
		PointToPlaneErrorMinimizer(aligned, map_points, map_normals, map_scores, filtered_sample_inds, H_icp);

		t[4] = std::clock();


		//////////////////////////////////////
		// Alignment with Motion distorsion //
		//////////////////////////////////////

		// Apply the incremental transformation found by ICP
		results.transform = H_icp * results.transform;

		// Align targets taking motion distortion into account
		if (params.motion_distortion)
		{
			size_t iphi = 0;
			for (auto& phi : phis)
			{
				float t = (phi - params.init_phi) / (phi1 - params.init_phi);
				Eigen::Matrix4d phi_H = pose_interp(t, params.init_transform, results.transform, 0);
				Eigen::Matrix3f phi_R = (phi_H.block(0, 0, 3, 3)).cast<float>();
				Eigen::Vector3f phi_T = (phi_H.block(0, 3, 3, 1)).cast<float>();
				aligned_mat.col(iphi) = (phi_R * targets_mat.col(iphi)) + phi_T;
				iphi++;
			}
		}
		else
		{
			Eigen::Matrix3f R_tot = (results.transform.block(0, 0, 3, 3)).cast<float>();
			Eigen::Vector3f T_tot = (results.transform.block(0, 3, 3, 1)).cast<float>();
			aligned_mat = (R_tot * targets_mat).colwise() + T_tot;
		}

		t[5] = std::clock();


		// Update all result matrices
		if (step == 0)
			results.all_transforms = Eigen::MatrixXd(results.transform);
		else
		{
			Eigen::MatrixXd temp(results.all_transforms.rows() + 4, 4);
			temp.topRows(results.all_transforms.rows()) = results.all_transforms;
			temp.bottomRows(4) = Eigen::MatrixXd(results.transform);
			results.all_transforms = temp;
		}

		t[5] = std::clock();

		///////////////////////
		// Check convergence //
		///////////////////////

		// Update variations
		if (!stop_cond && step > 0)
		{
			float avg_tot = (float)params.avg_steps;
			if (step == 1)
				avg_tot = 1.0;

			if (step > 0)
			{
				// Get last transformation variations
				Eigen::Matrix3d R2 = results.all_transforms.block(results.all_transforms.rows() - 4, 0, 3, 3);
				Eigen::Matrix3d R1 = results.all_transforms.block(results.all_transforms.rows() - 8, 0, 3, 3);
				Eigen::Vector3d T2 = results.all_transforms.block(results.all_transforms.rows() - 4, 3, 3, 1);
				Eigen::Vector3d T1 = results.all_transforms.block(results.all_transforms.rows() - 8, 3, 3, 1);
				R1 = R2 * R1.transpose();
				T1 = T2 - T1;
				float dT_b = T1.norm();
				float dR_b = acos((R1.trace() - 1) / 2);
				mean_dT += (dT_b - mean_dT) / avg_tot;
				mean_dR += (dR_b - mean_dR) / avg_tot;
			}
		}

		// Stop condition
		if (!stop_cond && step > params.avg_steps)
		{
			if (mean_dT < params.transDiffThresh && mean_dR < params.rotDiffThresh)
			{
				// Do not stop right away. Have a last few averaging steps
				stop_cond = true;
				max_it = step + params.avg_steps;
			}
		}

		// Last call, average the last transformations
		if (step > max_it - 2)
		{
			Eigen::Matrix4d mH = Eigen::Matrix4d::Identity(4, 4);
			for (size_t s = 0; s < params.avg_steps; s++)
			{
				Eigen::Matrix4d H = results.all_transforms.block(results.all_transforms.rows() - 4 * (1 + s), 0, 4, 4);
				mH = pose_interp(1.0 / (float)(s + 1), mH, H, 0);
			}
			results.transform = mH;
			results.all_transforms.block(results.all_transforms.rows() - 4, 0, 4, 4) = mH;
		}


		///////////// DEBUG /////////////

		//cout << "***********************" << endl;
		//for (size_t i = 0; i < t.size() - 1; i++)
		//{
		//	double duration = 1000 * (t[i + 1] - t[i]) / (double)CLOCKS_PER_SEC;
		//	cout << clock_str[i] << duration << " ms" << endl;
		//}
		//cout << " - - - - - - - - - - - - - ";
		//for (size_t i = 0; i < t.size() - 1; i++)
		//{
		//	double duration = 1000 * (t[i + 1] - t[i]) / (double)CLOCKS_PER_SEC;
		//	cout << duration << " ";
		//}
		//cout << endl;
		//cout << "***********************" << endl;

		//Eigen::Matrix3f R = H.block(0, 0, 3, 3);
		//Eigen::Vector3f T = H.block(0, 3, 3, 1);
		//cout << "dT = " << endl << T << endl;
		//cout << "dR = " << endl << T << endl;


		//if (step % 3 == 0)
		//{
		//	char buffer[100];
		//	sprintf(buffer, "cc_aligned_%03d.ply", (int)step * 0);
		//	save_cloud(string(buffer), aligned, phis);
		//}


	}


	// Final regularisation for good measure
	//for (size_t b = 0; b < B; b++)
	//{
	//	Eigen::Matrix4d dH = Eigen::Matrix4d::Identity(4, 4);
	//	for (size_t bb = b; bb < b + B; bb++)
	//	{
	//		size_t bb_0 = bb % B;
	//		dH = dH * results.transforms[bb_0];
	//		cout << " " << bb_0;
	//	}

	//	cout << endl << "dH" << b << " = " << endl;
	//	cout << dH << endl;
	//}

	//vector<float> H_w(B, 1.0f);
	//if (true)
	//	H_w[0] = 0;
	//average_poses(results.transforms, H_w);

	//for (size_t b = 0; b < B; b++)
	//{
	//	Eigen::Matrix4d dH = Eigen::Matrix4d::Identity(4, 4);
	//	for (size_t bb = b; bb < b + B; bb++)
	//	{
	//		size_t bb_0 = bb % B;
	//		dH = dH * results.transforms[bb_0];
	//		cout << " " << bb_0;
	//	}

	//	cout << endl << "dH" << b << " = " << endl;
	//	cout << dH << endl;
	//}





}


void PointToMapICP(vector<PointXYZ>& tgt_pts,
	vector<float>& tgt_w,
	PointMap& map,
	ICP_params& params,
	ICP_results& results)
{
	// Parameters
	// **********

	size_t N = tgt_pts.size();
	float max_pair_d2 = params.max_pairing_dist * params.max_pairing_dist;
	float max_planar_d = params.max_planar_dist;
	size_t first_steps = params.avg_steps / 2 + 1;

	// Get angles phi of each points for motion distorsion
	vector<float> phis;
	float phi1 = 0;
	if (params.motion_distortion)
	{
		phis.reserve(tgt_pts.size());
		for (auto& p : tgt_pts)
		{
			phis.push_back(fmod(3 * M_PI / 2 - atan2(p.y, p.x), 2 * M_PI));
			if (phis.back() > phi1)
				phi1 = phis.back();
		}
	}

	// Create search parameters
	nanoflann::SearchParams search_params;
	//search_params.sorted = false;

	// Start ICP loop
	// **************

	// Matrix of original data (only shallow copy of ref clouds)
	Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> map_mat((float*)map.cloud.pts.data(), 3, map.cloud.pts.size());

	// Aligned points (Deep copy of targets)
	vector<PointXYZ> aligned(tgt_pts);

	// Matrix for original/aligned data (Shallow copy of parts of the points vector)
	Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> targets_mat((float*)tgt_pts.data(), 3, N);
	Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> aligned_mat((float*)aligned.data(), 3, N);

	// Apply initial transformation
	Eigen::Matrix3f R_init = (params.init_transform.block(0, 0, 3, 3)).cast<float>();
	Eigen::Vector3f T_init = (params.init_transform.block(0, 3, 3, 1)).cast<float>();
	aligned_mat = (R_init * targets_mat).colwise() + T_init;
	results.transform = params.init_transform;

	// Random generator
	default_random_engine generator;
	discrete_distribution<int> distribution(tgt_w.begin(), tgt_w.end());

	// Init result containers
	Eigen::Matrix4d H_icp;
	results.all_rms.reserve(params.max_iter);
	results.all_plane_rms.reserve(params.max_iter);


	// Convergence varaibles
	float mean_dT = 0;
	float mean_dR = 0;
	size_t max_it = params.max_iter;
	bool stop_cond = false;

	vector<clock_t> t(6);
	vector<string> clock_str;
	clock_str.push_back("Random_Sample ... ");
	clock_str.push_back("KNN_search ...... ");
	clock_str.push_back("Optimization .... ");
	clock_str.push_back("Regularization .. ");
	clock_str.push_back("Result .......... ");

	for (size_t step = 0; step < max_it; step++)
	{
		/////////////////
		// Association //
		/////////////////

		// Pick random queries (use unordered set to ensure uniqueness)
		vector<pair<size_t, size_t>> sample_inds;
		if (params.n_samples < N)
		{
			unordered_set<size_t> unique_inds;
			while (unique_inds.size() < params.n_samples)
				unique_inds.insert((size_t)distribution(generator));

			sample_inds = vector<pair<size_t, size_t>>(params.n_samples);
			size_t i = 0;
			for (const auto& ind : unique_inds)
			{
				sample_inds[i].first = ind;
				i++;
			}
		}
		else
		{
			sample_inds = vector<pair<size_t, size_t>>(N);
			for (size_t i = 0; i < N; i++)
			{
				sample_inds[i].first = i;
				i++;
			}
		}

		t[1] = std::clock();

		// Init neighbors container
		vector<float> nn_dists(sample_inds.size());

		// Find nearest neigbors
		// #pragma omp parallel for shared(max_neighbs) schedule(dynamic, 10) num_threads(n_thread)
		for (size_t i = 0; i < sample_inds.size(); i++)
		{
			nanoflann::KNNResultSet<float> resultSet(1);
			resultSet.init(&sample_inds[i].second, &nn_dists[i]);
			map.tree.findNeighbors(resultSet, (float*)&aligned[sample_inds[i].first], search_params);
		}

		t[2] = std::clock();


		///////////////////////
		// Distances metrics //
		///////////////////////

		// Erase sample_inds if dists is too big
		vector<pair<size_t, size_t>> filtered_sample_inds;
		filtered_sample_inds.reserve(sample_inds.size());
		float rms2 = 0;
		float prms2 = 0;
		for (size_t i = 0; i < sample_inds.size(); i++)
		{
			if (nn_dists[i] < max_pair_d2)
			{
				// Check planar distance (only after a few steps for initial alignment)
				PointXYZ diff = (map.cloud.pts[sample_inds[i].second] - aligned[sample_inds[i].first]);
				float planar_dist = abs(diff.dot(map.normals[sample_inds[i].second]));
				if (step < first_steps || planar_dist < max_planar_d)
				{
					// Keep samples
					filtered_sample_inds.push_back(sample_inds[i]);

					// Update pt2pt rms
					rms2 += nn_dists[i];

					// update pt2pl rms
					prms2 += planar_dist;
				}

			}
		}
		// Compute RMS
		results.all_rms.push_back(sqrt(rms2 / (float)filtered_sample_inds.size()));
		results.all_plane_rms.push_back(sqrt(prms2 / (float)filtered_sample_inds.size()));

		t[3] = std::clock();


		//////////////////
		// Optimization //
		//////////////////

		// Minimize error
		PointToPlaneErrorMinimizer(aligned, map.cloud.pts, map.normals, map.scores, filtered_sample_inds, H_icp);

		t[4] = std::clock();


		//////////////////////////////////////
		// Alignment with Motion distorsion //
		//////////////////////////////////////

		// Apply the incremental transformation found by ICP
		results.transform = H_icp * results.transform;

		// Align targets taking motion distortion into account
		if (params.motion_distortion)
		{
			size_t iphi = 0;
			for (auto& phi : phis)
			{
				float t = (phi - params.init_phi) / (phi1 - params.init_phi);
				Eigen::Matrix4d phi_H = pose_interp(t, params.init_transform, results.transform, 0);
				Eigen::Matrix3f phi_R = (phi_H.block(0, 0, 3, 3)).cast<float>();
				Eigen::Vector3f phi_T = (phi_H.block(0, 3, 3, 1)).cast<float>();
				aligned_mat.col(iphi) = (phi_R * targets_mat.col(iphi)) + phi_T;
				iphi++;
			}
		}
		else
		{
			Eigen::Matrix3f R_tot = (results.transform.block(0, 0, 3, 3)).cast<float>();
			Eigen::Vector3f T_tot = (results.transform.block(0, 3, 3, 1)).cast<float>();
			aligned_mat = (R_tot * targets_mat).colwise() + T_tot;
		}

		t[5] = std::clock();


		// Update all result matrices
		if (step == 0)
			results.all_transforms = Eigen::MatrixXd(results.transform);
		else
		{
			Eigen::MatrixXd temp(results.all_transforms.rows() + 4, 4);
			temp.topRows(results.all_transforms.rows()) = results.all_transforms;
			temp.bottomRows(4) = Eigen::MatrixXd(results.transform);
			results.all_transforms = temp;
		}

		t[5] = std::clock();

		///////////////////////
		// Check convergence //
		///////////////////////

		// Update variations
		if (!stop_cond && step > 0)
		{
			float avg_tot = (float)params.avg_steps;
			if (step == 1)
				avg_tot = 1.0;

			if (step > 0)
			{
				// Get last transformation variations
				Eigen::Matrix3d R2 = results.all_transforms.block(results.all_transforms.rows() - 4, 0, 3, 3);
				Eigen::Matrix3d R1 = results.all_transforms.block(results.all_transforms.rows() - 8, 0, 3, 3);
				Eigen::Vector3d T2 = results.all_transforms.block(results.all_transforms.rows() - 4, 3, 3, 1);
				Eigen::Vector3d T1 = results.all_transforms.block(results.all_transforms.rows() - 8, 3, 3, 1);
				R1 = R2 * R1.transpose();
				T1 = T2 - T1;
				float dT_b = T1.norm();
				float dR_b = acos((R1.trace() - 1) / 2);
				mean_dT += (dT_b - mean_dT) / avg_tot;
				mean_dR += (dR_b - mean_dR) / avg_tot;
			}
		}

		// Stop condition
		if (!stop_cond && step > params.avg_steps)
		{
			if (mean_dT < params.transDiffThresh && mean_dR < params.rotDiffThresh)
			{
				// Do not stop right away. Have a last few averaging steps
				stop_cond = true;
				max_it = step + params.avg_steps;

				// For these last steps, reduce the max distance (half of wall thickness)
				max_planar_d = 0.08;
			}
		}

		// Last call, average the last transformations
		if (step > max_it - 2)
		{
			Eigen::Matrix4d mH = Eigen::Matrix4d::Identity(4, 4);
			for (size_t s = 0; s < params.avg_steps; s++)
			{
				Eigen::Matrix4d H = results.all_transforms.block(results.all_transforms.rows() - 4 * (1 + s), 0, 4, 4);
				mH = pose_interp(1.0 / (float)(s + 1), mH, H, 0);
			}
			results.transform = mH;
			results.all_transforms.block(results.all_transforms.rows() - 4, 0, 4, 4) = mH;
		}


		///////////// DEBUG /////////////

		//cout << "***********************" << endl;
		//for (size_t i = 0; i < t.size() - 1; i++)
		//{
		//	double duration = 1000 * (t[i + 1] - t[i]) / (double)CLOCKS_PER_SEC;
		//	cout << clock_str[i] << duration << " ms" << endl;
		//}
		//cout << " - - - - - - - - - - - - - ";
		//for (size_t i = 0; i < t.size() - 1; i++)
		//{
		//	double duration = 1000 * (t[i + 1] - t[i]) / (double)CLOCKS_PER_SEC;
		//	cout << duration << " ";
		//}
		//cout << endl;
		//cout << "***********************" << endl;

		//Eigen::Matrix3f R = H.block(0, 0, 3, 3);
		//Eigen::Vector3f T = H.block(0, 3, 3, 1);
		//cout << "dT = " << endl << T << endl;
		//cout << "dR = " << endl << T << endl;


		//if (step % 3 == 0)
		//{
		//	char buffer[100];
		//	sprintf(buffer, "cc_aligned_%03d.ply", (int)step * 0);
		//	save_cloud(string(buffer), aligned, phis);
		//}


	}


	// Final regularisation for good measure
	//for (size_t b = 0; b < B; b++)
	//{
	//	Eigen::Matrix4d dH = Eigen::Matrix4d::Identity(4, 4);
	//	for (size_t bb = b; bb < b + B; bb++)
	//	{
	//		size_t bb_0 = bb % B;
	//		dH = dH * results.transforms[bb_0];
	//		cout << " " << bb_0;
	//	}

	//	cout << endl << "dH" << b << " = " << endl;
	//	cout << dH << endl;
	//}

	//vector<float> H_w(B, 1.0f);
	//if (true)
	//	H_w[0] = 0;
	//average_poses(results.transforms, H_w);

	//for (size_t b = 0; b < B; b++)
	//{
	//	Eigen::Matrix4d dH = Eigen::Matrix4d::Identity(4, 4);
	//	for (size_t bb = b; bb < b + B; bb++)
	//	{
	//		size_t bb_0 = bb % B;
	//		dH = dH * results.transforms[bb_0];
	//		cout << " " << bb_0;
	//	}

	//	cout << endl << "dH" << b << " = " << endl;
	//	cout << dH << endl;
	//}





}


