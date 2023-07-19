#!/usr/bin/env python3

import numpy as np

from sklearn.model_selection import train_test_split
from torch.utils.data import DataLoader, Dataset

from .bag_reader import read_raw_data


def process_rdc_samples(rdc_samples, mag_norm_mean=None, mag_norm_std=None):
    magnitude = np.abs(rdc_samples)
    phase = np.angle(rdc_samples)
    magnitude_log = np.log1p(magnitude)

    if mag_norm_mean is not None and mag_norm_std is not None:
        magnitude_mean = mag_norm_mean
        magnitude_std = mag_norm_std
    else:
        magnitude_mean = np.mean(magnitude_log)
        magnitude_std = np.std(magnitude_log)

    # normalization
    magnitude_normalized = (magnitude_log - magnitude_mean) / magnitude_std
    phase_normalized = (phase + np.pi) / (2 * np.pi)

    # concatenate along the last dimension
    result = np.concatenate([magnitude_normalized[..., np.newaxis],
                             phase_normalized[..., np.newaxis]], axis=-1)
    # (N, C, D, H, W)
    result = result.transpose(0, 4, 1, 2, 3)

    return result, magnitude_mean, magnitude_std


class RdcTrainingDataset(Dataset):
    def __init__(self, rdc_samples, occupancy_grids):
        self.X, self.mag_norm_mean, self.mag_norm_std = process_rdc_samples(rdc_samples)
        self.Y = occupancy_grids

    def __getitem__(self, index):
        return self.X[index], self.Y[index]

    def __len__(self):
        return len(self.X)


class RdcTestingDataset(Dataset):
    def __init__(self, rdc_samples, occupancy_grids, mag_norm_mean, mag_norm_std):
        self.X, _, _ = process_rdc_samples(rdc_samples, mag_norm_mean=mag_norm_mean, mag_norm_std=mag_norm_std)
        self.Y = occupancy_grids

    def __getitem__(self, index):
        return self.X[index], self.Y[index]

    def __len__(self):
        return len(self.X)


def split_dataset(x_total, y_total):
    N = len(x_total)  # total number of samples
    total_idx = np.arange(N)
    train_indices, remain_indices = train_test_split(total_idx, test_size=0.4, random_state=42)
    validate_indices, test_indices = train_test_split(remain_indices, test_size=0.5, random_state=42)

    x_train, x_valid, x_test = x_total[train_indices], x_total[validate_indices], x_total[test_indices]
    y_train, y_valid, y_test = y_total[train_indices], y_total[validate_indices], y_total[test_indices]

    return x_train, x_valid, x_test, y_train, y_valid, y_test


def get_dataset(
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
    rdc_frames, diff_occupancy_grids = read_raw_data(
        dataset_dir=dataset_dir, rdc_bag_filename=rdc_bag_filename, diff_bag_filename=diff_bag_filename,
        rdc_topic=rdc_topic, update_map_pcl_topic=update_map_pcl_topic, update_map_odds_topic=update_map_odds_topic,
        diff_map_pcl_topic=diff_map_pcl_topic, diff_map_odds_topic=diff_map_odds_topic,
        use_cash=use_cash, visualize=visualize
    )
    print('Raw input shape:', rdc_frames.shape)
    print('Raw output shape:', diff_occupancy_grids.shape)
    # Input shape: (990, 12, 128, 128)
    # Output shape: (990, 32, 32, 24)

    x_train, x_valid, x_test, y_train, y_valid, y_test = split_dataset(
        x_total=rdc_frames,
        y_total=diff_occupancy_grids
    )
    train_dataset = RdcTrainingDataset(x_train, y_train)
    valid_dataset = RdcTestingDataset(
        x_valid, y_valid,
        mag_norm_mean=train_dataset.mag_norm_mean,
        mag_norm_std=train_dataset.mag_norm_std
    )
    test_dataset = RdcTestingDataset(
        x_test, y_test,
        mag_norm_mean=train_dataset.mag_norm_mean,
        mag_norm_std=train_dataset.mag_norm_std
    )
    print('Train input shape:', train_dataset.X.shape)
    print('Validate input shape:', valid_dataset.X.shape)
    print('Test input shape:', test_dataset.X.shape)

    train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)
    valid_loader = DataLoader(valid_dataset, batch_size=32, shuffle=False)
    test_loader = DataLoader(test_dataset, batch_size=32, shuffle=False)

    return train_loader, valid_loader, test_loader
