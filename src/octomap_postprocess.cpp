#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/MarkerArray.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

#include "octree_diff.h" 


std::pair<sensor_msgs::PointCloud2, visualization_msgs::MarkerArray> octreeToPointCloud2(const octomap::OcTree& tree, const std::string& frame_id, const ros::Time& stamp, double marker_lifetime = 0.1)
{
    // Extract occupied cells as an octomap::Pointcloud
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    visualization_msgs::MarkerArray marker_array;
    size_t id = 0;

    // Convert octomap::Pointcloud to pcl::PointCloud
    
    for(octomap::OcTree::leaf_iterator it = tree.begin_leafs(), end=tree.end_leafs(); it!= end; ++it) {
        // If the node is occupied, add it to the point cloud.
        if(tree.isNodeOccupied(*it)){
            pclCloud.points.push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
            
	    // Create marker for each occupied node
	    visualization_msgs::Marker marker;
	    marker.lifetime = ros::Duration(marker_lifetime);
	    marker.header.frame_id = frame_id;
	    marker.header.stamp = stamp;
	    marker.ns = "map";
	    marker.id = id++;
	    marker.type = visualization_msgs::Marker::CUBE;
	    marker.action = visualization_msgs::Marker::ADD;
	    marker.pose.position.x = it.getX();
	    marker.pose.position.y = it.getY();
	    marker.pose.position.z = it.getZ();
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

	    // Add marker to MarkerArray
	    marker_array.markers.push_back(marker);
	}
    }

    // Convert pcl::PointCloud to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(pclCloud, cloud);
    cloud.header.frame_id = frame_id;
    cloud.header.stamp = stamp;
    
    return std::make_pair(cloud, marker_array);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_postprocess_node");
    ros::NodeHandle nh("~");

    std::string input_bag_path, input_topic;
    double marker_lifetime;
    nh.getParam("input_bag_path", input_bag_path);
    nh.getParam("input_topic", input_topic);
    nh.getParam("marker_lifetime", marker_lifetime);
    
    std::string output_bag_path;
    std::string extension = ".bag";
    if (input_bag_path.size() > extension.size() &&
        input_bag_path.substr(input_bag_path.size() - extension.size()) == extension)
    {
        // If the input path ends with ".bag", replace it with "_diff.bag"
        output_bag_path = input_bag_path.substr(0, input_bag_path.size() - extension.size()) + "_diff.bag";
    } else {
        // If it doesn't, just append "_diff.bag" to the input path
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

    rosbag::View view(input_bag, rosbag::TopicQuery(input_topic));

    octomap::OcTree* prev_tree = nullptr;
    BOOST_FOREACH(rosbag::MessageInstance const msg, view) {
        octomap_msgs::Octomap::ConstPtr i_msg = msg.instantiate<octomap_msgs::Octomap>();
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
		 
		 auto [update_pcl, update_array] = octreeToPointCloud2(diff_trees.first, frame_id, stamp, marker_lifetime);
                auto [diff_pcl, diff_array] = octreeToPointCloud2(diff_trees.second, frame_id, stamp, marker_lifetime);
                
   		 output_bag.write(input_topic + "/update", msg.getTime(), update_tree_msg);
                output_bag.write(input_topic + "/update/pcl", msg.getTime(), update_pcl);
                output_bag.write(input_topic + "/update/array", msg.getTime(), update_array);
                
                output_bag.write(input_topic + "/diff", msg.getTime(), diff_tree_msg);
                output_bag.write(input_topic + "/diff/pcl", msg.getTime(), diff_pcl);
                output_bag.write(input_topic + "/diff/array", msg.getTime(), diff_array);
                
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

