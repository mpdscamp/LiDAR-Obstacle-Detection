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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function

	// For max iterations 
	for (int i = 0; i < maxIterations; i++) {
		std::unordered_set<int> inliersResultTemp;

		while (inliersResultTemp.size() < 2) {
			inliersResultTemp.insert(rand() % cloud->points.size());
		}

		auto itr = inliersResultTemp.begin();
		float x[2], y[2];
		for (int i = 0; i < 2; i++, itr++) {
			x[i] = cloud->points[*itr].x;
			y[i] = cloud->points[*itr].y;
		}

		float A = (y[0] - y[1]);
		float B = (x[1] - x[0]);
		float C = (x[0] * y[1] - x[1] * y[0]);

		for (size_t i = 0; i < cloud->points.size(); ++i) {
			if (inliersResultTemp.count(i) > 0) {
				continue;
			}

			float distance = fabs(A * cloud->points[i].x + B * cloud->points[i].y + C) / sqrt(A*A + B*B);

			if (distance < distanceTol) {
				inliersResultTemp.insert(i);
			}
		}

		if (inliersResultTemp.size() > inliersResult.size()) {
            inliersResult = inliersResultTemp;
        }
	}
	
	return inliersResult;
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function

	// For max iterations 
	for (size_t i = 0; i < maxIterations; i++) {
		std::unordered_set<int> inliersResultTemp;

		while (inliersResultTemp.size() < 3) {
			inliersResultTemp.insert(rand() % cloud->points.size());
		}

		auto itr = inliersResultTemp.begin();
		float point[3][3];
		for (int j = 0; j < 3; j++, itr++) {
			point[j][0] = cloud->points[*itr].x;
			point[j][1] = cloud->points[*itr].y;
			point[j][2] = cloud->points[*itr].z;
		}

		float vector[2][3];
		for (int j = 0; j < 2; j++) {
			for (int k = 0; k < 3; k++) {
				vector[j][k] = point[j+1][k] - point[0][k];
			}
		}

		float crossProduct[3];
		crossProduct[0] = vector[0][1] * vector[1][2] - vector[0][2] * vector[1][1];
		crossProduct[1] = vector[0][2] * vector[1][0] - vector[0][0] * vector[1][2];
		crossProduct[2] = vector[0][0] * vector[1][1] - vector[0][1] * vector[1][0];

		float D;
		for (int j = 0; j < 3; j++) {
			D -= crossProduct[j] * point[0][j];
		}

		for (size_t j = 0; j < cloud->points.size(); ++j) {
			if (inliersResultTemp.count(j) > 0) {
				continue;
			}

			float A = crossProduct[0], B = crossProduct[1], C = crossProduct[2];
			
			float distance = fabs(A * cloud->points[j].x + B * cloud->points[j].y + C * cloud->points[j].z + D) / sqrt(A*A + B*B + C*C);

			if (distance < distanceTol) {
				inliersResultTemp.insert(j);
			}
		}

		if (inliersResultTemp.size() > inliersResult.size()) {
            inliersResult = inliersResultTemp;
        }
	}
	
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 50, 0.5);

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
