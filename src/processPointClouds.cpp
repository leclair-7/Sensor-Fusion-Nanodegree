// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes,filterRes,filterRes);

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT> );
    vg.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT> );
    pcl::CropBox<PointT> regionOfInterestGen(true);
    regionOfInterestGen.setMin(minPoint);
    regionOfInterestGen.setMax(maxPoint);
    regionOfInterestGen.setInputCloud(cloud_filtered);
    regionOfInterestGen.filter(*cloud_region);

    std::vector<int>indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-2,-3,-1,1));
    roof.setMax(Eigen::Vector4f(3,3,1,1));
    roof.setInputCloud(cloud_region);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for ( int i : indices)
    	inliers->indices.push_back(i);

    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_region);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter (*cloud_region);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
    return cloud_region;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstacles_cloud(new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT> );

  typename pcl::ExtractIndices<PointT> extract;

  for( int idx : inliers->indices)
  {
	  plane_cloud->points.push_back(cloud->points.at(idx));
  }

  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*obstacles_cloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles_cloud, plane_cloud);
  return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

    seg.setOptimizeCoefficients(true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setInputCloud (cloud);
    seg.segment(*inliers, *coefficients);
    
    if (inliers->indices.size() <= 1)
    {
        std::cout << "RANSAC returned 1 or less points on plane segmentation, check input cloud\n";
    }

    std::cout << "Got the inliers, time to write to a pointer..\n\n\n";
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    
    return segResult;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	// TODO: Fill in this function

	// For max iterations
	size_t cloudNumPts = cloud->points.size();

	while( maxIterations-- )
	{
		std::unordered_set<int> inliersTest;
		size_t idx1 = rand() % cloudNumPts;
		size_t idx2 = rand() % cloudNumPts;
		size_t idx3 = rand() % cloudNumPts;

		// v1 to v2 and v1 to v3
		double x1 = cloud->points.at(idx1).x;
		double y1 = cloud->points.at(idx1).y;
		double z1 = cloud->points.at(idx1).z;

		double x2 = cloud->points.at(idx2).x;
		double y2 = cloud->points.at(idx2).y;
		double z2 = cloud->points.at(idx2).z;

		double x3 = cloud->points.at(idx3).x;
		double y3 = cloud->points.at(idx3).y;
		double z3 = cloud->points.at(idx3).z;

		// v1 goes from pt1 to pt2
		double v1x = x2-x1;
		double v1y = y2-y1;
		double v1z = z2-z1;

		// v2 goes from pt1 to pt3
		double v2x = x3-x1;
		double v2y = y3-y1;
		double v2z = z3-z1;

		// get cross-prod v1xv2
		double A = (v1y * v2z) - (v2y*v1z);
		double B = -1 * ( (v1x * v2z) - (v2x-v1z) );
		double C = v1x*v2y - v1y*v2x;
		double D = -1 * (x1*A + y1*B + z1*C);

		for ( size_t j=0; j < cloud->points.size() ; j++)
		{
			if  ( j == idx1 || j == idx2 || j== idx3 )
			{
				continue;
			}
			double px = cloud->points.at(j).x;
			double py = cloud->points.at(j).y;
			double pz = cloud->points.at(j).z;

			double cVal = sqrt(A*A + B*B + C*C);
			// don't divide by 0
			if (cVal < .000001)
			{
				continue;
			}

			double distToLine = fabs( A*px + B*py + C*pz + D ) / cVal;

			if ( distToLine < distanceTol)
			{
				inliersTest.insert(j);
			}
		}
		if ( inliersTest.size() > inliersResult.size() )
		{
			inliersResult = inliersTest;
		}
	}
	// CHECK Randomly sample subset and fit line
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	return inliersResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::getPlaneAndOtherPcPoints(typename pcl::PointCloud<PointT>::Ptr aCloud,std::unordered_set<int> aInliers)
{
	typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new typename pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new typename pcl::PointCloud<PointT>());

	for(int index = 0; index < aCloud->points.size(); index++)
	{
		PointT point = aCloud->points[index];
		if(aInliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	typename std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> res(cloudOutliers,cloudInliers);
	return res;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneRansac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
	std::unordered_set<int> resIdxPlane = Ransac3d(cloud, maxIterations, distanceThreshold);
	typename std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> clustersPlanePair = getPlaneAndOtherPcPoints(cloud,resIdxPlane);
	return clustersPlanePair;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
    	typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    		cloud_cluster->push_back ((*cloud)[*pit]); //*
    	cloud_cluster->width = cloud_cluster->size ();
    	cloud_cluster->height = 1;
    	cloud_cluster->is_dense = true;

    	clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
