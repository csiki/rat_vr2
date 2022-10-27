import numpy as np
import torch


class OmniTransferOpt(torch.nn.Module):
    def __init__(self, def_d):
        super().__init__()
        self.tm = torch.tensor(
            [[-np.sin(np.pi / 3), np.cos(np.pi / 3), def_d],
             [0, -1, def_d],
             [np.sin(np.pi / 3), np.cos(np.pi / 3), def_d]], dtype=torch.float32)
        self.inv_tm_v = torch.nn.Parameter(torch.inverse(self.tm))

        # self.tm_lin = torch.nn.Linear(3, 3, bias=True)
        # self.tm_lin.weight.data = self.tm_lin.weight.data * 0 + self.tm

    def forward(self, wheel_speeds):
        # wheel_speeds = torch.matmul(self.tm_v, desired_speed)
        motion_pred = torch.matmul(self.inv_tm_v, wheel_speeds)
        return motion_pred
