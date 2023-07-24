#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include "std_msgs/Float64MultiArray.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2_eigen/tf2_eigen.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

#include "octree_diff.h"
#include "pcl_sampling.h"


Eigen::Affine3d poseMsgToEigen(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::Pose pose = msg->pose.pose;
    geometry_msgs::Transform transform;
    transform.translation.x = pose.position.x;
    transform.translation.y = pose.position.y;
    transform.translation.z = pose.position.z;
    transform.rotation = pose.orientation;
    Eigen::Affine3d eigenTransform = tf2::transformToEigen(transform);
    return eigenTransform.inverse();
}


std::tuple<std_msgs::Float64MultiArray, pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZ>, visualization_msgs::MarkerArray> octreeToPointCloud(
    const octomap::OcTree& tree,
    const std::string& frame_id,
    const ros::Time& stamp,
    const Eigen::Affine3d& center,
    double marker_lifetime = 0.1,
    double voxel_size = 0.5
) {
    pcl::PointCloud<pcl::PointXYZ> pclUnoccupied, pclCloudCentered, pclCloud;
    visualization_msgs::MarkerArray markerArray;
    std_msgs::Float64MultiArray occupancyOdds;
    occupancyOdds.data.clear();
    size_t id = 0;
    
    for(octomap::OcTree::leaf_iterator it = tree.begin_leafs(), end=tree.end_leafs(); it!= end; ++it) {
        Eigen::Vector3d pointOrigEig(it.getX(), it.getY(), it.getZ());
        Eigen::Vector3d pointCenteredEig = center * pointOrigEig;
        pcl::PointXYZ pointOrig(pointOrigEig[0], pointOrigEig[1], pointOrigEig[2]);
        pcl::PointXYZ pointCentered(pointCenteredEig[0], pointCenteredEig[1], pointCenteredEig[2]);
        pclUnoccupied.points.push_back(pointCentered);
        occupancyOdds.data.push_back(it->getLogOdds());
        
        if(tree.isNodeOccupied(*it)){
            pclCloud.points.push_back(pointOrig);
            pclCloudCentered.points.push_back(pointCentered);
            
            // Create marker for each occupied node
            visualization_msgs::Marker marker;
            marker.lifetime = ros::Duration(marker_lifetime);
            marker.header.frame_id = frame_id;
            marker.header.stamp = stamp;
            marker.ns = "map";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = pointOrig.x;
            marker.pose.position.y = pointOrig.y;
            marker.pose.position.z = pointOrig.z;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = tree.getNodeSize(it.getDepth());
            marker.scale.y = tree.getNodeSize(it.getDepth());
            marker.scale.z = tree.getNodeSize(it.getDepth());
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            markerArray.markers.push_back(marker);
        }
    }

/**
    sensor_msgs::PointCloud2 cloudCentered;
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(pclCloudCentered, cloudCentered);
    pcl::toROSMsg(pclCloud, cloud);
    cloudCentered.header.frame_id = frame_id;
    cloudCentered.header.stamp = stamp;
    cloud.header.frame_id = frame_id;
    cloud.header.stamp = stamp;
**/
    
    return std::make_tuple(occupancyOdds, pclUnoccupied, pclCloudCentered, pclCloud, markerArray);
}


sensor_msgs::PointCloud2 pclToMsg(pcl::PointCloud<pcl::PointXYZ>& cloud, const std::string& frame_id, const ros::Time& stamp) {
    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg(cloud, cloudMsg);
    cloudMsg.header.frame_id = frame_id;
    cloudMsg.header.stamp = stamp;
    return cloudMsg;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_postprocess_node");
    ros::NodeHandle nh("~");

// PROCESS PARAMETERS
    std::string input_bag_path, input_topic, odometry_topic;
    double marker_lifetime = 0.1, voxel_radius = 0;
    int target_N_points = 0;
    if (!nh.getParam("input_bag_path", input_bag_path) || input_bag_path.empty()) {
        ROS_ERROR("No input_bag_path provided");
    }
    if (!nh.getParam("input_topic", input_topic) || input_topic.empty()) {
        ROS_ERROR("No input_topic provided");
    }
    if (!nh.getParam("odometry_topic", odometry_topic) || odometry_topic.empty()) {
        ROS_ERROR("No odometry_topic provided");
    }
    if (nh.getParam("marker_lifetime", marker_lifetime) && marker_lifetime < 0) {
        ROS_ERROR("marker_lifetime can't be negative");
    }
    if (nh.getParam("voxel_radius", voxel_radius) != nh.getParam("target_N_points", target_N_points)) {
        ROS_ERROR("Either both voxel_radius and target_N_points must be specified or neither");
    } 
    if (voxel_radius < 0) {
        ROS_ERROR("voxel_radius can't be negative");
    }
    if (target_N_points < 0) {
        ROS_ERROR("target_N_points can't be negative");
    }

// PROCESS INPUT BAG
    std::string output_bag_path;
    std::string extension = ".bag";
    if (input_bag_path.size() > extension.size() &&
        input_bag_path.substr(input_bag_path.size() - extension.size()) == extension)
    {
        output_bag_path = input_bag_path.substr(0, input_bag_path.size() - extension.size()) + "_diff.bag";
    } else {
        output_bag_path = input_bag_path + "_diff.bag";
    }

    rosbag::Bag input_bag, output_bag;
    try {
        input_bag.open(input_bag_path, rosbag::bagmode::Read);
        output_bag.open(output_bag_path, rosbag::bagmode::Write);
    } catch(rosbag::BagException& e) {
        ROS_ERROR("Error opening bag files: %s", e.what());
        return 1;
    }
    rosbag::View view(input_bag, rosbag::TopicQuery({input_topic, odometry_topic}));

    // check if required topics exist
    bool input_topic_found = false, odometry_topic_found = false;
    BOOST_FOREACH(const rosbag::ConnectionInfo *info, view.getConnections()) {
      if (info->topic == input_topic) {
        input_topic_found = true;
      }
      else if (info->topic == odometry_topic) {
        odometry_topic_found = true;
      }
    }
    if (!input_topic_found) {
        ROS_ERROR("Required topic '%s' not found in file '%s'", input_topic.c_str(), input_bag_path.c_str());
        return 1;
    }
    if (!odometry_topic_found) {
        ROS_ERROR("Required topic '%s' not found in file '%s'", odometry_topic.c_str(), input_bag_path.c_str());
        return 1;
    }
    
// PROCESS DATA
    Eigen::Affine3d odometry_position = Eigen::Affine3d::Identity();
    octomap::OcTree* prev_tree = nullptr;

    BOOST_FOREACH(rosbag::MessageInstance const msg, view) {
        octomap_msgs::Octomap::ConstPtr i_msg = msg.instantiate<octomap_msgs::Octomap>();
        nav_msgs::Odometry::ConstPtr o_msg = msg.instantiate<nav_msgs::Odometry>();

        if (o_msg != nullptr) {
            odometry_position = poseMsgToEigen(o_msg);
        }

        if (i_msg != nullptr) {
            octomap::OcTree* current_tree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*i_msg));
            if (prev_tree != nullptr) {
                std::string frame_id = i_msg->header.frame_id;
                ros::Time stamp = i_msg->header.stamp;

                std::pair<octomap::OcTree, octomap::OcTree> diff_trees = calcOctreeDiff(*prev_tree, *current_tree);
                octomap_msgs::Octomap diff_tree_msg, update_tree_msg;
                octomap_msgs::binaryMapToMsg(diff_trees.first, update_tree_msg);
                octomap_msgs::binaryMapToMsg(diff_trees.second, diff_tree_msg);
                
                diff_tree_msg.header.frame_id = frame_id;
                diff_tree_msg.header.stamp = stamp;
                update_tree_msg.header.frame_id = frame_id;
                update_tree_msg.header.stamp = stamp;

                auto [updateOccupancyOddsMsg, updatePclUnoccupied, updatePclCentered, updatePcl, updateArray] = octreeToPointCloud(diff_trees.first, frame_id, stamp, odometry_position, marker_lifetime);
                auto [diffOccupancyOddsMsg, diffPclUnoccupied, diffPclCentered, diffPcl, diffArray] = octreeToPointCloud(diff_trees.second, frame_id, stamp, odometry_position, marker_lifetime);
                ROS_INFO("Update tree: %lu points", updatePclCentered.points.size());
                ROS_INFO("Diff tree: %lu points", diffPclCentered.points.size());
                
                sensor_msgs::PointCloud2 updatePclUnoccupiedMsg = pclToMsg(updatePclUnoccupied, frame_id, stamp);
                sensor_msgs::PointCloud2 updatePclCenteredMsg = pclToMsg(updatePclCentered, frame_id, stamp);
                sensor_msgs::PointCloud2 updatePclMsg = pclToMsg(updatePcl, frame_id, stamp);
                sensor_msgs::PointCloud2 diffPclUnoccupiedMsg = pclToMsg(diffPclUnoccupied, frame_id, stamp);
                sensor_msgs::PointCloud2 diffPclCenteredMsg = pclToMsg(diffPclCentered, frame_id, stamp);
                sensor_msgs::PointCloud2 diffPclMsg = pclToMsg(diffPcl, frame_id, stamp);

                output_bag.write(input_topic + "/update", msg.getTime(), update_tree_msg);
                output_bag.write(input_topic + "/update/pcl_centered", msg.getTime(), updatePclCenteredMsg);
                output_bag.write(input_topic + "/update/pcl", msg.getTime(), updatePclMsg);
                output_bag.write(input_topic + "/update/array", msg.getTime(), updateArray);
                output_bag.write(input_topic + "/update/pcl_centered_full", msg.getTime(), updatePclUnoccupiedMsg);
                output_bag.write(input_topic + "/update/pcl_occupancy_odds", msg.getTime(), updateOccupancyOddsMsg);
                
                output_bag.write(input_topic + "/diff", msg.getTime(), diff_tree_msg);
                output_bag.write(input_topic + "/diff/pcl_centered", msg.getTime(), diffPclCenteredMsg);
                output_bag.write(input_topic + "/diff/pcl", msg.getTime(), diffPclMsg);
                output_bag.write(input_topic + "/diff/array", msg.getTime(), diffArray);
                output_bag.write(input_topic + "/diff/pcl_centered_full", msg.getTime(), diffPclUnoccupiedMsg);
                output_bag.write(input_topic + "/diff/pcl_occupancy_odds", msg.getTime(), diffOccupancyOddsMsg);
               
                
                if (target_N_points > 0) {
                    if (target_N_points > updatePclCentered.points.size()) {
                        ROS_INFO("Upsampling update tree");
                        pcl::PointCloud<pcl::PointXYZ> updatePclCenteredUpsampled = updatePclCentered + upsamplePointCloud(updatePclCentered, voxel_radius, target_N_points);
                        sensor_msgs::PointCloud2 updatePclCenteredUpsampledMsg = pclToMsg(updatePclCenteredUpsampled, frame_id, stamp);
                        output_bag.write(input_topic + "/update/pcl_centered_upsampled", msg.getTime(), updatePclCenteredUpsampledMsg);
                    }
                    /**
                    if (target_N_points > diff_pcl_centered.points.size()) {
                        pcl::PointCloud<pcl::PointXYZ> diffPclCenteredUpsampled = diff_pcl_centered + upsamplePointCloud(diff_pcl_centered, voxel_radius, target_N_points);
                        sensor_msgs::PointCloud2 diffPclCenteredUpsampledMsg = pclToMsg(diffPclCenteredUpsampled, frame_id, stamp);
                        output_bag.write(input_topic + "/diff/pcl_centered_upsampled", msg.getTime(), diffPclCenteredUpsampledMsg);
                    }
                    **/
                }
            }
            if (prev_tree != nullptr) delete prev_tree;
            prev_tree = current_tree;
        }
    }

    if (prev_tree != nullptr) delete prev_tree;
    input_bag.close();
    output_bag.close();
    return 0;
}

