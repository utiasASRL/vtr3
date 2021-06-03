#include "vtr_lidar/pointmap/pointmap.h"

void PointMapPython::update(vector<PointXYZ>& points0,
	vector<PointXYZ>& normals0,
	vector<float>& scores0)
{
	// Initialize variables
	// ********************

	// New limits of the map
	PointXYZ minCorner = min_point(points0);
	PointXYZ maxCorner = max_point(points0);
	PointXYZ originCorner = floor(minCorner * (1 / dl) - PointXYZ(1, 1, 1)) * dl;

	// Check old limits of the map
	if (points.size() > 0)
	{
		PointXYZ oldMinCorner = min_point(points);
		PointXYZ oldMaxCorner = max_point(points);
		PointXYZ oldOriginCorner = floor(oldMinCorner * (1 / dl) - PointXYZ(1, 1, 1)) * dl;
		originCorner = min_point(originCorner, oldOriginCorner);
		maxCorner = max_point(maxCorner, oldMaxCorner);
	}

	// Dimensions of the grid
	size_t sampleNX = (size_t)floor((maxCorner.x - originCorner.x) / dl) + 2;
	size_t sampleNY = (size_t)floor((maxCorner.y - originCorner.y) / dl) + 2;

	// Create the sampled map
	// **********************

	unordered_map<size_t, MapVoxelData> samples;

	//// USe following to know if we need to reserve before inserting elements
	samples.reserve(points.size() + points0.size());
	//std::cout << "current max_load_factor: " << samples.max_load_factor() << std::endl;
	//std::cout << "current size: " << samples.size() << std::endl;
	//std::cout << "current bucket_count: " << samples.bucket_count() << std::endl;
	//std::cout << "current load_factor: " << samples.load_factor() << std::endl;

	// Add existing map points to the hashmap.
	if (points.size() > 0)
	{
		init_samples(originCorner, maxCorner, samples);
	}

	// Add new points to the hashmap
	add_samples(points0, normals0, scores0, originCorner, maxCorner, samples);

	// Convert hmap to vectors
	points.reserve(samples.size());
	normals.reserve(samples.size());
	scores.reserve(samples.size());
	counts.reserve(samples.size());
	size_t i = 0;
	size_t iX, iY, iZ, centroidIdx;
	for (auto& v : samples)
	{
		PointXYZ centroid = v.second.centroid * (1.0 / v.second.count);
		iX = (size_t)floor((centroid.x - originCorner.x) / dl);
		iY = (size_t)floor((centroid.y - originCorner.y) / dl);
		iZ = (size_t)floor((centroid.z - originCorner.z) / dl);
		centroidIdx = iX + sampleNX * iY + sampleNX * sampleNY * iZ;

		if (v.second.occupied && centroidIdx == v.first)
		{
			v.second.normal *= 1.0 / (sqrt(v.second.normal.sq_norm()) + 1e-6);
			if (i < points.size())
			{
				points[i] = centroid;
				normals[i] = v.second.normal;
				scores[i] = v.second.score;
				counts[i] = v.second.count;
			}
			else
			{
				points.push_back(v.second.centroid * (1.0 / v.second.count));
				normals.push_back(v.second.normal);
				scores.push_back(v.second.score);
				counts.push_back(v.second.count);
			}
			i++;
		}
	}
}


void PointMapPython::init_samples(const PointXYZ originCorner,
	const PointXYZ maxCorner,
	unordered_map<size_t, MapVoxelData>& samples)
{
	// Dimensions of the grid
	size_t sampleNX = (size_t)floor((maxCorner.x - originCorner.x) / dl) + 2;
	size_t sampleNY = (size_t)floor((maxCorner.y - originCorner.y) / dl) + 2;

	// Initialize variables
	size_t i = 0;
	size_t iX, iY, iZ, mapIdx;

	for (auto& p : points)
	{
		// Position of point in sample map
		iX = (size_t)floor((p.x - originCorner.x) / dl);
		iY = (size_t)floor((p.y - originCorner.y) / dl);
		iZ = (size_t)floor((p.z - originCorner.z) / dl);

		// Update the point cell
		mapIdx = iX + sampleNX * iY + sampleNX * sampleNY * iZ;
		samples.emplace(mapIdx, MapVoxelData(p * counts[i], normals[i] * counts[i], scores[i], counts[i]));
		i++;
	}
}



void PointMapPython::add_samples(const vector<PointXYZ>& points0,
	const vector<PointXYZ>& normals0,
	const vector<float>& scores0,
	const PointXYZ originCorner,
	const PointXYZ maxCorner,
	unordered_map<size_t, MapVoxelData>& samples)
{
	// Dimensions of the grid
	size_t sampleNX = (size_t)floor((maxCorner.x - originCorner.x) / dl) + 2;
	size_t sampleNY = (size_t)floor((maxCorner.y - originCorner.y) / dl) + 2;

	// Initialize variables
	float r2 = dl * 1.5;
	r2 *= r2;
	size_t i = 0;
	size_t iX, iY, iZ, mapIdx;

	for (auto& p : points0)
	{
		// Position of point in sample map
		iX = (size_t)floor((p.x - originCorner.x) / dl);
		iY = (size_t)floor((p.y - originCorner.y) / dl);
		iZ = (size_t)floor((p.z - originCorner.z) / dl);

		// Update the adjacent cells
		for (size_t ix = iX - 1; ix < iX + 2; ix++)
		{

			for (size_t iy = iY - 1; iy < iY + 2; iy++)
			{

				for (size_t iz = iZ - 1; iz < iZ + 2; iz++)
				{
					// Find distance to cell center
					mapIdx = ix + sampleNX * iy + sampleNX * sampleNY * iz;
					PointXYZ cellCenter(ix + 0.5, iy + 0.5, iz + 0.5);
					cellCenter = cellCenter * dl + originCorner;
					float d2 = (cellCenter - p).sq_norm();

					// Update barycenter if in range
					if (d2 < r2)
					{
						if (samples.count(mapIdx) < 1)
							samples.emplace(mapIdx, MapVoxelData(p));
						else
							samples[mapIdx].update_centroid(p);
					}
				}
			}
		}

		// Update the point cell
		mapIdx = iX + sampleNX * iY + sampleNX * sampleNY * iZ;
		samples[mapIdx].update_normal(scores0[i], normals0[i]);
		i++;
	}
}

