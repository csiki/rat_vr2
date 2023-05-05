import time
import sched
import numpy as np
from typing import Callable, Any, Union, Tuple
from collections import deque


class PlayerMovement:  # in-game movement tracking

    MIN_HISTORY = 6
    HISTORY_SIZE = 500
    SMOOTH_DT = .1  # sec; < 0
    AXES = 3

    def __init__(self, hist_size: int = HISTORY_SIZE, smooth_dt: float = SMOOTH_DT):

        # poll these to get curr val
        self.pos = np.zeros(PlayerMovement.AXES)
        self.vel = np.zeros(PlayerMovement.AXES)
        self.acc = np.zeros(PlayerMovement.AXES)

        def _smoothing(dts):
            # windowing of previous pos recordings to compute vel or acc
            # in the time window given, weights recordings with longer dt higher; same as in motion
            past = np.cumsum(dts[::-1])[::-1]  # 10, 7, 5, 2, 1
            within_t = past - smooth_dt  # smooth_dt = 6; 4, 1, -1, -4, -5
            t_occupied = np.clip(dts - within_t, 0, None)  # 0, 1, 4, 5, 6
            t_occupied = np.min([t_occupied, dts], axis=0)  # 0, 1, 3, 1, 1
            weight = t_occupied / max(1e-4, t_occupied.sum())  # normalize, avoid nans
            return weight

        self.smoothing = _smoothing
        self._ts = deque(maxlen=hist_size)  # time points and..
        self._ps = deque(maxlen=hist_size)  # positions to calc vel/acc

    def loop(self, pos: np.ndarray, vel: np.ndarray = np.empty(0), acc: np.ndarray = np.empty(0)):
        tnow = time.time()
        self.pos = self._reg_pos(pos, tnow)  # in-game position (x, y, angle); angle in rad

        # the last elements of velocity and acceleration are partially calculated if not provided
        #   e.g. for DOOM angle velocity cannot be read from game variables (that easy)
        self.vel[:vel.size] = vel  # in-game velocity (x/s, y/s, angle/s)
        self.vel[vel.size:] = self._calc_vel(tnow)[vel.size:] if vel.size < PlayerMovement.AXES else []
        self.acc[:acc.size] = acc  # in-game acceleration (x/s^2, y/s^2, angle/s^2)
        self.acc[acc.size:] = self._calc_acc(tnow)[acc.size:] if acc.size < PlayerMovement.AXES else []

    def _reg_pos(self, p: np.ndarray, tnow: float):
        self._ts.append(tnow)
        self._ps.append(p)
        return p

    def _calc_vel(self, tnow: float):  # in-case get_vel is not available
        if len(self._ts) < PlayerMovement.MIN_HISTORY:  # not enough
            return np.zeros(PlayerMovement.AXES)

        dts = np.diff(np.array(self._ts))
        dps = np.diff(np.array(self._ps), axis=0)
        return (self.smoothing(dts)[:, None] * dps / dts[:, None]).mean(axis=0)

    def _calc_acc(self, tnow):
        if len(self._ts) < PlayerMovement.MIN_HISTORY:  # not enough
            return np.zeros(PlayerMovement.AXES)

        ts = np.array(self._ts)
        dts = np.diff(ts)
        dps = np.diff(np.array(self._ps), axis=0)
        return (self.smoothing(dts[1:])[:, None] * np.diff(dps, axis=0) / dts[1:, None]).mean(axis=0)


class Feedback:
    def __init__(self, movement: PlayerMovement, setpoint: np.ndarray = None, rel_setpoint: np.ndarray = None):
        assert setpoint is None ^ rel_setpoint is None
        self.movement = movement
        self.setpoint = setpoint if setpoint else movement.pos + rel_setpoint

    def __call__(self):  # returns error from setpoint
        return self.setpoint - self.movement.pos

    def pos(self):
        return self.movement.pos

    def vel(self):
        return self.movement.vel

    def acc(self):
        return self.movement.acc
