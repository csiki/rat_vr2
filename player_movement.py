import time
import sched
import numpy as np
from typing import Callable, Any, Union, Tuple
from collections import deque


class PlayerMovement:  # in-game movement tracking

    MIN_HISTORY = 6
    HISTORY_SIZE = 100
    SMOOTH_DT = -.1  # sec; < 0
    AXES = 3

    def __init__(self, do_calc_vel: bool = False, do_calc_acc: bool = False,
                 hist_size: int = HISTORY_SIZE, smooth_dt: float = SMOOTH_DT):
        self.do_calc_vel = do_calc_vel
        self.do_calc_acc = do_calc_acc

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
            weight = t_occupied / t_occupied.sum()  # normalize
            return weight

        self.calc_some = do_calc_vel is None or do_calc_acc is None
        self.smoothing = _smoothing
        self._ts = deque(maxlen=hist_size)  # time points and..
        self._ps = deque(maxlen=hist_size)  # positions to calc vel/acc

    def loop(self, pos: np.ndarray, vel: np.ndarray = None, acc: np.ndarray = None):
        tnow = time.time()
        self.pos = self._reg_pos(pos, tnow)  # in-game position (x, y, angle); angle in rad
        self.vel = vel if vel is not None else self._calc_vel(tnow)  # in-game velocity (x/s, y/s, angle/s)
        self.acc = acc if acc is not None else self._calc_acc(tnow)  # in-game acceleration (x/s^2, y/s^2, angle/s^2)

    def _reg_pos(self, p: np.ndarray, tnow: float):
        if self.calc_some:
            self._ts.append(tnow)
            self._ps.append(p)
        return p

    def _calc_vel(self, tnow: float):  # in-case get_vel is not available
        if len(self._ts) < PlayerMovement.MIN_HISTORY:  # not enough
            return np.zeros(3)

        ts = np.array(self._ts)
        dts = np.diff(ts)
        dps = np.diff(np.array(self._ps))
        return self.smoothing(dts) * dps / dts

    def _calc_acc(self, tnow):
        if len(self._ts) < PlayerMovement.MIN_HISTORY:  # not enough
            return np.zeros(3)

        ts = np.array(self._ts)
        dts = np.diff(ts)
        dps = np.diff(np.array(self._ps))
        return self.smoothing(ts[2:] - tnow) * np.diff(dps) / dts[1:]


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
