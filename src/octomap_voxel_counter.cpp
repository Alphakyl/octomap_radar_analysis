#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <std_msgs/Int32.h>

using namespace octomap;

ros::Publisher pub;

void octomap_cb(const octomap_msgs::Octomap::ConstPtr& msg){
    octomap::AbstractOcTree* abstract_octomap = binaryMsgToMap(*msg);
    octomap::OcTree* octomap_store = NULL;
    std_msgs::Int32 count;
    count.data = 0;

    if(abstract_octomap){
        octomap_store = dynamic_cast<octomap::OcTree*>(abstract_octomap);
    } else {
        ROS_ERROR("Error creating OcTree from received message");
    }
    if(octomap_store){
        octomap_store->expand();
        for(octomap::OcTree::leaf_iterator it = octomap_store->begin_leafs(),end = octomap_store->end_leafs(); it !=end; ++it){
            if(octomap_store->isNodeOccupied(*it)){
                count.data++;
            }
        }
        pub.publish(count);
    }
}



int main(int argc, char** argv){
    ros::init(argc,argv,"count_octomap_voxels");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Subscriber sub = nh.subscribe("/octomap_binary",10,octomap_cb);
    pub = nh.advertise<std_msgs::Int32>("octomap_size_count",1);
    ros::spin();
    return 0;
}