import time
import sched
import numpy as np
from typing import Callable, Any, Union, Tuple
from collections import deque


class PlayerMovement:

    MIN_MOV_HISTORY = 6
    MOV_HISTORY_SIZE = 100
    MOV_HISTORY_RELEVANCE_T = -.1  # sec
    AXES = 3

    def __init__(self, get_pos: Callable[[], np.ndarray],
                 get_vel: Callable[[], np.ndarray] = None, get_acc: Callable[[], np.ndarray] = None,
                 mov_hist_size: int = MOV_HISTORY_SIZE, mov_history_relevance_t: float = MOV_HISTORY_RELEVANCE_T):
        self._get_pos = get_pos
        self._get_vel = get_vel
        self._get_acc = get_acc

        # poll these to get curr val
        self.pos = np.zeros(PlayerMovement.AXES)
        self.vel = np.zeros(PlayerMovement.AXES)
        self.acc = np.zeros(PlayerMovement.AXES)

        def _smoothing(rel_ts):  # linear windowing of previous pos recordings to compute vel or acc
            w = np.clip(rel_ts - mov_history_relevance_t, 0, None)
            return w / w.sum()

        self.calc_some = get_vel is None or get_acc is None
        self.smoothing = _smoothing
        self._ts = deque(maxlen=mov_hist_size)  # time points and..
        self._ps = deque(maxlen=mov_hist_size)  # positions to calc vel/acc

    def loop(self):
        tnow = time.time()
        self.pos = self._reg_pos()
        self.vel = self._reg_vel(tnow)
        self.acc = self._reg_acc(tnow)

    def _reg_pos(self):  # returns in-game position (x, y, angle); angle in rad
        p = self._get_pos()
        if self.calc_some:
            self._ts.append(time.time())
            self._ps.append(p)
        return p

    def _reg_vel(self, tnow: float):  # returns in-game velocity (x/s, y/s, angle/s)
        return self._get_vel() if self._get_vel else self._calc_vel(tnow)

    def _reg_acc(self, tnow: float):  # returns in-game acceleration (x/s^2, y/s^2, angle/s^2)
        return self._get_acc() if self._get_acc else self._calc_acc(tnow)

    def _calc_vel(self, tnow: float):  # in-case get_vel is not available
        if len(self._ts) < PlayerMovement.MIN_MOV_HISTORY:  # not enough
            return np.zeros(3)

        ts = np.array(self._ts)
        dts = np.diff(ts)
        dps = np.diff(np.array(self._ps))
        return self.smoothing(ts[1:] - tnow) * dps / dts

    def _calc_acc(self, tnow):
        if len(self._ts) < PlayerMovement.MIN_MOV_HISTORY:  # not enough
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
