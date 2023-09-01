#!/usr/bin/env python

import torch
from torch import nn

import numpy as np
import matplotlib.pyplot as plt

# export PYTHONPATH=$PYTHONPATH:src/octomap_radar_analysis/src
from learning.dataset import get_dataset
from learning.model import BaseTransform


def get_device():
    if torch.cuda.is_available():
        print('GPU is available.')
        device = torch.device("cuda")
    else:
        print('GPU is not available, using CPU.')
        device = torch.device("cpu")
    return device


def test_model(test_loader, model, criterion, device):
    model.eval()
    test_loss = 0
    predicted_output = []

    with torch.no_grad():
        for data, target in test_loader:
            data, target = data.to(device), target.to(device)
            output = model(data)
            loss = criterion(output, target)
            test_loss += loss
            predicted_output.append(output.cpu().numpy())

    print(f'Testing loss: {test_loss}')
    return np.concatenate(predicted_output, axis=0)


def visualize_grids(
        true_grids, predicted_grids,
        odds_threshold=0.0, resolution=0.25,
        x_min_meters=-6, x_max_meters=6,
        y_min_meters=0, y_max_meters=8,
        z_min_meters=0, z_max_meters=4
):
    fig = plt.figure(figsize=(12, 6))
    cmap = plt.get_cmap('jet')

    # first subplot for true grids
    ax_true, ax_predicted = fig.add_subplot(121, projection='3d'), fig.add_subplot(122, projection='3d')

    for true_grid, predicted_grid in zip(true_grids, predicted_grids):
        try:
            for grid, ax in ((true_grid, ax_true), (predicted_grid, ax_predicted)):
                threshold_indices = np.where(grid >= odds_threshold)
                odds = grid[threshold_indices]
                xs, ys, zs = threshold_indices
                xs, ys, zs = xs * resolution + x_min_meters, ys * resolution + y_min_meters, zs * resolution + z_min_meters
                ax.scatter(xs, ys, zs, c=odds, s=1, cmap=cmap)
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.set_xlim([x_min_meters, x_max_meters])
                ax.set_ylim([y_min_meters, y_max_meters])
                ax.set_zlim([z_min_meters, z_max_meters])

            ax_true.set_title('True Grids')
            ax_predicted.set_title('Predicted Grids')
            plt.draw()
            plt.pause(1.5)
            ax_true.clear(), ax_predicted.clear()
        except KeyboardInterrupt:
            plt.close()
            break

    plt.close()


def main(visualize=True):
    _, _, test_loader = get_dataset(use_cash=True, visualize=False)
    device = get_device()
    model = BaseTransform().double().to(device)
    model.load_state_dict(torch.load('model.pt'))
    criterion = nn.L1Loss()

    print('Start testing')
    predicted_output = test_model(test_loader, model, criterion, device)
    print('Testing finished.')
    # print(test_loader.dataset.Y[0][0][0])
    # print(predicted_output[0][0][0])

    resolution_meters = 0.25
    x_min_meters = -4
    x_max_meters = 4
    y_min_meters = 0
    y_max_meters = 8
    z_min_meters = -4
    z_max_meters = 4
    if visualize:
        visualize_grids(
            true_grids=test_loader.dataset.Y, predicted_grids=predicted_output,
            odds_threshold=0, resolution=resolution_meters,
            x_min_meters=x_min_meters, x_max_meters=x_max_meters,
            y_min_meters=y_min_meters, y_max_meters=y_max_meters,
            z_min_meters=z_min_meters, z_max_meters=z_max_meters
        )


if __name__ == "__main__":
    main()
