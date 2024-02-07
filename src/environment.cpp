/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "render/render.h"
#include "processPointClouds.h"
#include "utils/kdtree.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#include <unordered_set>

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;

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

		float D = 0.0f;
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

void proximity(std::vector<std::vector<float>> points, int i, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol) {
	processed[i] = true;

	cluster.push_back(i);

	pcl::Indices nearbyPoints = tree->search(points[i], distanceTol);

	for (const int& nearbyPoint : nearbyPoints) {
		if (!processed[nearbyPoint]) {
			proximity(points, nearbyPoint, cluster, processed, tree, distanceTol);
		}
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);

    for (int i = 0; i < points.size(); i++) {
        if (!processed[i]) {
            std::vector<int> cluster;
            proximity(points, i, cluster, processed, tree, distanceTol);

            if (cluster.size() >= minSize && cluster.size() <= maxSize) {
                clusters.push_back(cluster);
            }
        }
    }

    return clusters;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
    srand(time(NULL));
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.25f, Eigen::Vector4f(-15, -5.5, -2.5f, 0.1f), Eigen::Vector4f(30, 7, 15, 1.0f));

    // Segmentation using RANSAC
    std::unordered_set<int> inliers = Ransac3D(filterCloud, 100, 0.2f);

	pcl::PointCloud<pcl::PointXYZI>::Ptr  planeCloud(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr obstCloud(new pcl::PointCloud<pcl::PointXYZI>());

	for(int index = 0; index < filterCloud->points.size(); index++)
	{
		pcl::PointXYZI point = filterCloud->points[index];
		if(inliers.count(index))
			planeCloud->points.push_back(point);
		else
			obstCloud->points.push_back(point);
	}

    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> points;
    for (int i = 0; i < obstCloud->points.size(); i++) {
        points.push_back({obstCloud->points[i].x, obstCloud->points[i].y, obstCloud->points[i].z});
        tree->insert(points[i], i);
    }

    std::vector<std::vector<int>> cloudClusters = euclideanCluster(points, tree, 0.45f, 9, 2000);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(std::vector<int> cluster : cloudClusters)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
        for(int indice: cluster) {
            pcl::PointXYZI point;
            point.x = obstCloud->points[indice].x;
            point.y = obstCloud->points[indice].y;
            point.z = obstCloud->points[indice].z;
            point.intensity = obstCloud->points[indice].intensity;
            clusterCloud->points.push_back(point);
        }
        renderPointCloud(viewer, clusterCloud,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);
        Box box = pointProcessorI->BoundingBox(clusterCloud);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }

    renderPointCloud(viewer, planeCloud, "planeCloud", Color(0, 1, 0));
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //CameraAngle setAngle = XY;
    initCamera(FPS, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    } 
}