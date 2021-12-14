import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np

class Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv = nn.Conv2d(3, 32, kernel_size=(3, 3))
        self.pool = nn.MaxPool2d(2, 2)
        self.fc = nn.Linear(5408, 2)

    def forward(self, x):
        x = self.pool(F.relu(self.conv(x)))
        x = torch.flatten(x, 1)  # flatten all dimensions except batch
        x = self.fc(x)
        return x

    def predict(self, x):
        if isinstance(x, np.ndarray):
            is_numpy = True
            x = torch.from_numpy(x).permute([0, 3, 1, 2])
        else:
            is_numpy = False
        output = self.forward(x)
        if is_numpy:
            return output.detach().numpy()
        else:
            return output
