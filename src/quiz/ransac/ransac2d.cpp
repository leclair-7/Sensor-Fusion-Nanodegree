/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include <vector>
#include <numeric>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <math.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac2d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	size_t cloudNumPts = cloud->points.size();

	std::vector<double>lineDistances;

	while( maxIterations-- )
	{
		std::unordered_set<int> inliersTest;
		size_t idx1 = rand() % cloudNumPts;
		size_t idx2 = rand() % cloudNumPts;

		double x1 =  cloud->points.at(idx1).x;
		double y1 =  cloud->points.at(idx1).y;
		double x2 =  cloud->points.at(idx2).x;
		double y2 =  cloud->points.at(idx2).y;

		double lA = y1-y2;
		double lB = x2-x1;
		double lC = (x1*y2 - x2*y1);

		for ( size_t j=0; j < cloud->points.size() ; j++)
		{
			if  ( j == idx1 || j == idx2 )
			{
				continue;
			}
			double px = cloud->points.at(j).x;
			double py = cloud->points.at(j).y;

			double cVal = sqrt(lA*lA + lB*lB);
			if (cVal < .000001 )
			{
				//std::cout << "Warning small sqrt\n";
				continue;
			}
			double distToLine = fabs( lA * px + lB * py + lC) / cVal;
			lineDistances.push_back(distToLine);


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

	// Return indicies of inliers from fitted line with most inliers
	double avgAllDist = std::accumulate(lineDistances.begin(), lineDistances.end(),0.0);
	avgAllDist /= lineDistances.size();
	
	std::cout << "avgDist to line " << avgAllDist << "\n";

	return inliersResult;
}

std::unordered_set<int> Ransac3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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

std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> getPlaneAndOtherPcPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr aCloud,std::unordered_set<int> aInliers)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < aCloud->points.size(); index++)
	{
		pcl::PointXYZ point = aCloud->points[index];
		if(aInliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	return std::make_pair(cloudInliers,cloudOutliers);
}

int main ()
{
	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3d(cloud, 50, .50);

	std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> inOutliers = getPlaneAndOtherPcPoints(cloud, inliers);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers = inOutliers.first;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers = inOutliers.second;

	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
