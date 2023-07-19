import torch.nn as nn


class BaseTransform(nn.Module):
    def __init__(self):
        super().__init__()
        self.layer1 = nn.Sequential(
            nn.Conv3d(2, 16, kernel_size=5, stride=1, padding=2),
            nn.ReLU(),
            nn.MaxPool3d(kernel_size=2, stride=2))
        self.layer2 = nn.Sequential(
            nn.Conv3d(16, 24, kernel_size=5, stride=1, padding=2),
            nn.ReLU(),
            nn.MaxPool3d(kernel_size=(1, 4, 4)))  # We only pool in the spatial dimensions

    def forward(self, x):
        out = self.layer1(x)
        out = self.layer2(out)
        return out
