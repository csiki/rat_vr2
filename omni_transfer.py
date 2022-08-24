import numpy as np
import torch

# d = (10. + 6.75) / 100.
# mx = np.array(
#     [[-np.sin(np.pi / 3), np.cos(np.pi / 3), d],
#      [0, -1, d],
#      [np.sin(np.pi / 3), np.cos(np.pi / 3), d]]
# )
#
# mxt=torch.tensor(mx)

# TODO input update? https://discuss.pytorch.org/t/how-to-add-cosine-similarity-score-in-cross-entropy-loss/64401


class OmniTransfer(torch.nn.Module):
    def __init__(self, def_d):
        super().__init__()
        d = (10. + 6.75) / 100.
        self.tm = torch.tensor(
            [[-np.sin(np.pi / 3), np.cos(np.pi / 3), d],
             [0, -1, d],
             [np.sin(np.pi / 3), np.cos(np.pi / 3), d]]
        )
        self.inv_tm_v = torch.nn.Parameter(torch.inverse(self.tm))

        # self.tm_lin = torch.nn.Linear(3, 3, bias=True)
        # self.tm_lin.weight.data = self.tm_lin.weight.data * 0 + self.tm

    def forward(self, wheel_speeds):
        # wheel_speeds = torch.matmul(self.tm_v, desired_speed)
        motion_pred = torch.matmul(self.inv_tm_v, wheel_speeds)
        return motion_pred


# trans = OmniTransfer(d)
#
# for i in range(50):
#     speed_test =
#     speed_pred = trans()
