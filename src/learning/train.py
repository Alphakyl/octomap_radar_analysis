#!/usr/bin/env python3


# export PYTHONPATH=$PYTHONPATH:src/octomap_radar_analysis/src
from learning.dataset import get_dataset
from learning.model import BaseTransform


if __name__ == "__main__":
    train_loader, valid_loader, test_loader = get_dataset()
    # parser = argparse.ArgumentParser(description="Adjust pointcloud sizes in a rosbag file.")
    # parser.add_argument("--file", type=str, help="Path to the rosbag file.")
    # parser.add_argument("--topic", type=str, help="Topic of the PointCloud2 messages.")
    # parser.add_argument("-N", type=int, help="Target number of points.")
    # args = parser.parse_args()
    #
    # main(args.bag_path, args.topic, args.target_points)
