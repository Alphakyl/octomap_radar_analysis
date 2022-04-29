#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

void odom_cb(const nav_msgs::OdometryConstPtr& odom_msg){
    static tf::TransformBroadcaster br;
    tf::Transform tf;
    geometry_msgs::Pose odom_msg_pose = odom_msg->pose.pose;

    tf::poseMsgToTF(odom_msg_pose,tf);
    tf::StampedTransform stamped_tf(tf,odom_msg->header.stamp,odom_msg->header.frame_id,"imu_viz_link");
    br.sendTransform(stamped_tf);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"odom2tf");
    ros::NodeHandle nh;

    ros::Subscriber sub_odom = nh.subscribe("/lidar_ground_truth",10,odom_cb);
    ros::spin();
    return 0;
}