/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

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
/*
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
    
	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	int numPoints = cloud->points.size();
	float x1, x2, y1, y2, x, y;
	float A, B, C, d;

	for(int i=0; i<maxIterations; i++)
	{
		std::unordered_set<int> inliers;
		while(inliers.size()<2)
		{
			inliers.insert(rand()%numPoints);
		}
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x; 
		y1 = cloud->points[*itr].y;
		itr++;
		x2 = cloud->points[*itr].x; 
		y2 = cloud->points[*itr].y;

		A = y1-y2;
		B = x2-x1;
		C = (x1*y2 - x2*y1);

		for(int j =0; j< numPoints; j++)
		{
			pcl::PointXYZ point = cloud->points[j];
			x = point.x; 
			y = point.y;

			if(inliers.count(j)>0)
				continue;
			
			d = fabs(A*x + B*y + C)/(sqrt(A*A + B*B));
			if(d < distanceTol)
			{
				inliers.insert(j);
			}

		}

		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}	
	}

	return inliersResult;

}
*/

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
    
	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	int numPoints = cloud->points.size();
	float x1, x2, x3, y1, y2, y3, z1, z2, z3, x, y, z;
	float A, B, C, D, dist;

	for(int i=0; i<maxIterations; i++)
	{
		std::unordered_set<int> inliers;
		while(inliers.size()<3)
		{
			inliers.insert(rand()%numPoints);
		}
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x; 
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x; 
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;

		itr++;
		x3 = cloud->points[*itr].x; 
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		A = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		B = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		C = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		D = -1 * (A*x1 + B*y1 + C*z1);

		for(int j =0; j< numPoints; j++)
		{
			pcl::PointXYZ point = cloud->points[j];
			x = point.x; 
			y = point.y;
			z = point.z;

			if(inliers.count(j)>0)
				continue;
			
			dist = fabs(A*x + B*y + C*z + D)/(sqrt(A*A + B*B + C*C));
			if(dist < distanceTol)
			{
				inliers.insert(j);
			}

		}

		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}	
	}

	return inliersResult;

}
/*
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
    
	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	int numPoints = cloud->points.size();
	float x1, x2, y1, y2, x, y;
	float A, B, C, d;

	for(int i=0; i<maxIterations; i++)
	{
		std::unordered_set<int> inliers;
		while(inliers.size()<2)
		{
			inliers.insert(rand()%numPoints);
		}
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x; 
		y1 = cloud->points[*itr].y;
		itr++;
		x2 = cloud->points[*itr].x; 
		y2 = cloud->points[*itr].y;

		A = y1-y2;
		B = x2-x1;
		C = (x1*y2 - x2*y1);

		for(int j =0; j< numPoints; j++)
		{
			pcl::PointXYZ point = cloud->points[j];
			x = point.x; 
			y = point.y;

			if(inliers.count(j)>0)
				continue;
			
			d = fabs(A*x + B*y + C)/(sqrt(A*A + B*B));
			if(d < distanceTol)
			{
				inliers.insert(j);
			}

		}

		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}	
	}

	return inliersResult;

}
*/

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 0.4);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


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
