#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/frustum_culling.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <iostream>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg){
    // std::cout << "Recieved Cloud" << std::endl;
    pcl::PCLPointCloud2* cloud_model = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloud_model_ptr(cloud_model);
    sensor_msgs::PointCloud2 output_msg;

	pcl_conversions::toPCL(*msg, *cloud_model);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_model,*temp_cloud);

    pcl::FrustumCulling<pcl::PointXYZ> fc;
    fc.setInputCloud(temp_cloud);
    fc.setVerticalFOV(45);
    fc.setHorizontalFOV(45);
    fc.setNearPlaneDistance(0.0);
    fc.setFarPlaneDistance(15);
    Eigen::Matrix4f radar_pose = Eigen::Matrix4f::Identity(4,4);
    radar_pose(0,0) = -1.0;
    radar_pose(2,2) = -1.0;
    // std::cout << radar_pose << std::endl;
    fc.setCameraPose(radar_pose);
    pcl::PointCloud<pcl::PointXYZ> target;
    fc.filter (target);

    pcl::toROSMsg(target,output_msg);
    output_msg.header.frame_id = msg->header.frame_id;
    output_msg.header.stamp = msg->header.stamp;
    pub.publish(output_msg);
}

int main(int argc, char** argv){
    ros::init(argc,argv,"point_cloud_angle_filter");
    ros::NodeHandle nh;
    ros::Subscriber sub;
    pub = nh.advertise<sensor_msgs::PointCloud2>("filterd_cloud",10);
    sub = nh.subscribe("in_cloud", 1, cloud_cb);
    ros::spin();
    return 0;
}