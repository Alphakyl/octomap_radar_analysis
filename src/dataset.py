#!/usr/bin/env python3


import os

import rosbag
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray
from dca1000_device.msg import RadarCubeMsg
import rospy


def process_messages(dataset_dir, rdc_bag_filename, diff_bag_filename, rdc_topic, map_topic, odds_topic):  # rdc_messages, map_messages, odds_messages):
    rdc_bag = rosbag.Bag(os.path.join(dataset_dir, rdc_bag_filename))
    map_bag = rosbag.Bag(os.path.join(dataset_dir, diff_bag_filename))

    map_frames_idx, rdc_frames_idx = [], []
    rdc_iter = rdc_bag.read_messages(topics=[rdc_topic])
    _, rdc_msg, _ = next(rdc_iter)

    for _, msg, _ in map_bag.read_messages(topics=[map_topic]):
        if msg.header.stamp
    # matched = []
    # j = 0
    # for i in range(len(messages1)):
    #     while j < len(messages2) and messages2[j].header.stamp < messages1[i].header.stamp:
    #         j += 1
    #     if j < len(messages2) and messages2[j].header.stamp == messages1[i].header.stamp:
    #         matched.append((messages1[i], messages2[j]))
    # return matched


def main(
        rdc_topic='/dca_node/data_cube',
        map_pcl_topic='/lidar_filtered/octomap_full/update/pcl_centered_full',
        map_odds_topic='/lidar_filtered/octomap_full/update/pcl_occupancy_odds'
):
    dataset_dir = '/home/arpg/mapping/coloradar'
    rdc_bag_filename = 'ec_hallways_run0.bag'
    diff_bag_filename = 'ec_hallways_run0_lidar_octomap_diff.bag'
    map_frames, map_odds, rdc_frames = process_messages(dataset_dir, rdc_bag_filename, diff_bag_filename)



if __name__ == "__main__":
    main()
    # parser = argparse.ArgumentParser(description="Adjust pointcloud sizes in a rosbag file.")
    # parser.add_argument("--file", type=str, help="Path to the rosbag file.")
    # parser.add_argument("--topic", type=str, help="Topic of the PointCloud2 messages.")
    # parser.add_argument("-N", type=int, help="Target number of points.")
    # args = parser.parse_args()
    #
    # main(args.bag_path, args.topic, args.target_points)
