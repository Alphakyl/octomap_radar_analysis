#!/bin/bash
NAME=/home/kyle/data/isrr_octomaps/ec_data/alt_tune/ec_hallways_run1

rosrun octomap_server octomap_saver ${NAME}_l.bt octomap_binary:=/lidar/octomap_binary
rosrun octomap_server octomap_saver ${NAME}_lf.bt octomap_binary:=/lidar_filtered/octomap_binary
rosrun octomap_server octomap_saver ${NAME}_r.bt octomap_binary:=/radar_point_cloud/octomap_binary
rosrun octomap_server octomap_saver ${NAME}_rl.bt octomap_binary:=/radar_point_cloud_lidar_processing/octomap_binary