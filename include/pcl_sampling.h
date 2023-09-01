#ifndef PCL_SAMPLING_H
#define PCL_SAMPLING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZ> upsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>& pclCloud, double radius, int nPoints);

#endif
