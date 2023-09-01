import torch
import torch.nn as nn


class BaseTransform(nn.Module):
    def __init__(self):
        super().__init__()
        # [32, 2, 12, 128, 128]
        self.layer1 = nn.Sequential(
            nn.Conv3d(2, 16, kernel_size=5, stride=1, padding=2),   # [32, 16, 12, 128, 128]
            nn.MaxPool3d(kernel_size=2, stride=2))  # [32, 16, 6, 64, 64]

        self.layer2 = nn.Conv3d(16, 1, kernel_size=1)  # [32, 1, 6, 64, 64]
        # squeeze
        # [32, 6, 64, 64]
        self.layer3 = nn.Conv2d(6, 32, kernel_size=4, stride=2, padding=1)   # [32, 32, 32, 32]

    def forward(self, x):
        # [32, 2, 12, 128, 128]
        out = self.layer1(x)
        # print("Shape after layer1: ", out.shape)
        # [32, 16, 6, 64, 64]
        out = self.layer2(out)
        # print("Shape after layer2: ", out.shape)
        # [32, 1, 6, 64, 64]
        out = out.squeeze(1)
        # print("Shape after layer3: ", out.shape)
        # [32, 6, 64, 64]
        out = self.layer3(out)
        # print("Shape after layer4: ", out.shape)
        # [32, 32, 32, 32]
        out = torch.sigmoid(out)
        return out
