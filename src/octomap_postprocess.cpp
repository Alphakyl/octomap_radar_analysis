#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

#include "octree_diff.h" 

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_postprocess_node");
    ros::NodeHandle nh("~");

    std::string input_bag_path, input_topic;
    nh.getParam("input_bag_path", input_bag_path);
    nh.getParam("input_topic", input_topic);
    
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
                std::pair<octomap::OcTree, octomap::OcTree> diff_trees = calcOctreeDiff(*prev_tree, *current_tree);
                octomap_msgs::Octomap diff_tree_msg, update_tree_msg;
                octomap_msgs::binaryMapToMsg(diff_trees.second, diff_tree_msg);
                octomap_msgs::binaryMapToMsg(diff_trees.first, update_tree_msg);

                output_bag.write(input_topic + "/diff", msg.getTime(), diff_tree_msg);
                output_bag.write(input_topic + "/update", msg.getTime(), update_tree_msg);
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

