#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import os
import pickle
from tqdm import tqdm

import rosbag
import rospy
from sensor_msgs import point_cloud2 as pc2


def map_pcl_to_grid(
        pcl_frames, odds_frames,
        resolution_meters=0.25,
        x_min_meters=-6, x_max_meters=6,
        y_min_meters=0, y_max_meters=8,
        z_min_meters=0, z_max_meters=4
):
    assert len(pcl_frames) == len(odds_frames)

    # calculate the dimensions of the grid
    x_bins = int((x_max_meters - x_min_meters) / resolution_meters)
    y_bins = int((y_max_meters - y_min_meters) / resolution_meters)
    z_bins = int((z_max_meters - z_min_meters) / resolution_meters)

    occupancy_grids = []
    visual_grids = []  # additional grids for visualization
    saved_points_count = 0
    discarded_points_count = 0

    for frame, odds in zip(pcl_frames, odds_frames):
        assert len(frame) == len(odds)
        occupancy_grid = np.zeros((x_bins, y_bins, z_bins))
        x_vis, y_vis, z_vis, odd_vis = [], [], [], []

        # calculate the grid cell for each point in the frame
        for point, point_odds in zip(frame, odds):
            x, y, z = point
            x_idx = int((x - x_min_meters) / resolution_meters)
            y_idx = int((y - y_min_meters) / resolution_meters)
            z_idx = int((z - z_min_meters) / resolution_meters)
            # check if indices are within the grid dimensions
            if 0 <= x_idx < x_bins and 0 <= y_idx < y_bins and 0 <= z_idx < z_bins:
                # update the occupancy odds for the grid cell
                occupancy_grid[x_idx, y_idx, z_idx] = point_odds
                x_vis.append(x)
                y_vis.append(y)
                z_vis.append(z)
                odd_vis.append(point_odds)
                saved_points_count += 1
            else:
                discarded_points_count += 1
        occupancy_grids.append(occupancy_grid)
        visual_grids.append(np.array([np.array(x_vis), np.array(y_vis), np.array(z_vis), np.array(odd_vis)]))

    print('Saved points to grid:', saved_points_count)
    print('Discarded points:', discarded_points_count)

    return np.array(occupancy_grids), visual_grids


def process_messages(
        dataset_dir, rdc_bag_filename, diff_bag_filename,
        rdc_topic, map_update_topic, map_diff_topic, update_odds_topic, diff_odds_topic,
        use_cash=False, cash_file='dataset_backup.pkl',
        resolution_meters=0.25, x_min_meters=-6, x_max_meters=6, y_min_meters=0, y_max_meters=8, z_min_meters=0, z_max_meters=4
):
    cash_filepath = os.path.join(dataset_dir, cash_file)
    if use_cash and os.path.isfile(cash_filepath):
        print('Using cash')
        with open(cash_filepath, 'rb') as f:
            rdc_frames, update_occupancy_grids, update_visual_grids, diff_occupancy_grids, diff_visual_grids = pickle.load(f)
        return rdc_frames, update_occupancy_grids, update_visual_grids, diff_occupancy_grids, diff_visual_grids

    rdc_bag = rosbag.Bag(os.path.join(dataset_dir, rdc_bag_filename))
    map_bag = rosbag.Bag(os.path.join(dataset_dir, diff_bag_filename))

    map_frames_idx, rdc_frames_idx = [], []
    map_iter = enumerate(map_bag.read_messages(topics=[map_update_topic]))
    map_idx, (_, map_msg, _) = next(map_iter)
    prev_time_diff = rospy.Duration.from_sec(99999999)

    for rdc_idx, (_, rdc_msg, _) in tqdm(enumerate(rdc_bag.read_messages(topics=[rdc_topic]))):
        while True:
            current_time_diff = abs(rdc_msg.header.stamp - map_msg.header.stamp)
            if current_time_diff < prev_time_diff:
                prev_time_diff = current_time_diff
                try:
                    map_idx, (_, map_msg, _) = next(map_iter)
                except StopIteration:
                    break
            else:
                rdc_frames_idx.append(rdc_idx)
                map_frames_idx.append(map_idx - 1)
                prev_time_diff = rospy.Duration.from_sec(99999999)
                break

    map_frames_idx, rdc_frames_idx = np.array(map_frames_idx), np.array(rdc_frames_idx)
    rdc_frames, map_update_frames, map_diff_frames, update_odds_frames, diff_odds_frames = [], [], [], [], []

    for topic, msg, t in tqdm(rdc_bag.read_messages(topics=[rdc_topic])):
        samples = np.array(msg.samples)
        samples = samples[:-1:2] + 1j * samples[1::2]
        reshaped_samples = samples.reshape(msg.num_tx * msg.num_rx, msg.num_chirps_per_frame, msg.num_adc_samples_per_chirp)
        rdc_frames.append(reshaped_samples)
    for topic, msg, t in tqdm(map_bag.read_messages(topics=[map_update_topic, map_diff_topic, update_odds_topic, diff_odds_topic])):
        if topic == map_update_topic:
            point_generator = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            frame = np.array([np.array(list(point)) for point in point_generator])
            map_update_frames.append(frame)
        elif topic == map_diff_topic:
            point_generator = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            frame = np.array([np.array(list(point)) for point in point_generator])
            map_diff_frames.append(frame)
        elif topic == update_odds_topic:
            update_odds_frames.append(np.array(msg.data))
        elif topic == diff_odds_topic:
            diff_odds_frames.append(np.array(msg.data))

    rdc_bag.close()
    map_bag.close()
    rdc_frames = np.array([rdc_frames[i] for i in rdc_frames_idx])
    map_update_frames = [map_update_frames[i] for i in map_frames_idx]
    map_diff_frames = [map_diff_frames[i] for i in map_frames_idx]
    update_odds_frames = [update_odds_frames[i] for i in map_frames_idx]
    diff_odds_frames = [diff_odds_frames[i] for i in map_frames_idx]
    print('RDC frames:', len(rdc_frames))
    print('Map frames:', len(map_update_frames))

    update_occupancy_grids, update_visual_grids = map_pcl_to_grid(
        pcl_frames=map_update_frames, odds_frames=update_odds_frames,
        resolution_meters=resolution_meters,
        x_min_meters=x_min_meters, x_max_meters=x_max_meters,
        y_min_meters=y_min_meters, y_max_meters=y_max_meters,
        z_min_meters=z_min_meters, z_max_meters=z_max_meters
    )
    diff_occupancy_grids, diff_visual_grids = map_pcl_to_grid(
        pcl_frames=map_diff_frames, odds_frames=diff_odds_frames,
        resolution_meters=resolution_meters,
        x_min_meters=x_min_meters, x_max_meters=x_max_meters,
        y_min_meters=y_min_meters, y_max_meters=y_max_meters,
        z_min_meters=z_min_meters, z_max_meters=z_max_meters
    )

    with open(cash_filepath, 'wb') as f:
        pickle.dump([rdc_frames, update_occupancy_grids, update_visual_grids, diff_occupancy_grids, diff_visual_grids], f)

    return rdc_frames, update_occupancy_grids, update_visual_grids, diff_occupancy_grids, diff_visual_grids


def visualize_grid(
        grids, odds_threshold=0,
        x_min_meters=-6, x_max_meters=6,
        y_min_meters=0, y_max_meters=8,
        z_min_meters=0, z_max_meters=4
):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for i, grid in enumerate(grids):
        xs, ys, zs, odds = grid
        target_points = np.where(odds >= odds_threshold)
        xs, ys, zs, odds = xs[target_points], ys[target_points], zs[target_points], odds[target_points]
        cmap = plt.get_cmap('jet')
        ax.scatter(xs, ys, zs, c=odds, s=1, cmap=cmap)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim([x_min_meters, x_max_meters])
        ax.set_ylim([y_min_meters, y_max_meters])
        ax.set_zlim([z_min_meters, z_max_meters])
        plt.draw()
        plt.pause(1)
        ax.clear()
    plt.close()


def read_raw_data(
        dataset_dir='mapping/coloradar',
        rdc_bag_filename='ec_hallways_run0.bag',
        diff_bag_filename='ec_hallways_run0_lidar_octomap_diff.bag',
        rdc_topic='/dca_node/data_cube',
        update_map_pcl_topic='/lidar_filtered/octomap_full/update/pcl_centered_full',
        update_map_odds_topic='/lidar_filtered/octomap_full/update/pcl_occupancy_odds',
        diff_map_pcl_topic='/lidar_filtered/octomap_full/diff/pcl_centered_full',
        diff_map_odds_topic='/lidar_filtered/octomap_full/diff/pcl_occupancy_odds',
        use_cash=True, visualize=False
):
    resolution_meters = 0.25
    x_min_meters = -4
    x_max_meters = 4
    y_min_meters = 0
    y_max_meters = 8
    z_min_meters = -4
    z_max_meters = 4

    rdc_frames, update_occupancy_grids, update_visual_grids, diff_occupancy_grids, diff_visual_grids = process_messages(
        os.path.join(os.path.expanduser('~'), dataset_dir), rdc_bag_filename, diff_bag_filename,
        rdc_topic=rdc_topic, map_update_topic=update_map_pcl_topic, update_odds_topic=update_map_odds_topic,
        map_diff_topic=diff_map_pcl_topic, diff_odds_topic=diff_map_odds_topic,
        use_cash=use_cash,
        resolution_meters=resolution_meters,
        x_min_meters=x_min_meters, x_max_meters=x_max_meters,
        y_min_meters=y_min_meters, y_max_meters=y_max_meters,
        z_min_meters=z_min_meters, z_max_meters=z_max_meters
    )

    if visualize:
        visualize_grid(
            diff_visual_grids, odds_threshold=-0.4,
            x_min_meters=x_min_meters, x_max_meters=x_max_meters,
            y_min_meters=y_min_meters, y_max_meters=y_max_meters,
            z_min_meters=z_min_meters, z_max_meters=z_max_meters
        )

    return rdc_frames, diff_occupancy_grids
