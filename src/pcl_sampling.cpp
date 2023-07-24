#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


pcl::PointCloud<pcl::PointXYZ> upsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>& pclCloud, double radius, int targetNPoints) {
    pcl::PointCloud<pcl::PointXYZ> newCloud;
    int realNPoints = pclCloud.points.size();
    int currentNPoints = pclCloud.points.size();
    
    // Random number generator
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-radius, radius);

    while (currentNPoints < targetNPoints) {
        // Choose a random existing point
        int i = rand() % realNPoints;

        // Create a new point inside the voxel of the chosen point
        pcl::PointXYZ newPoint = pclCloud.points[i];
        newPoint.x += distribution(generator);
        newPoint.y += distribution(generator);
        newPoint.z += distribution(generator);

        newCloud.points.push_back(newPoint);
        currentNPoints++;
    }
    
    ROS_INFO("Sampled %lu points", newCloud.points.size());

    return newCloud;
}
