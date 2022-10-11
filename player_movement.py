import time
import sched
import numpy as np
from typing import Callable, Any, Union, Tuple
from collections import deque


class PlayerMovement:  # in-game movement tracking

    MIN_MOV_HISTORY = 6
    MOV_HISTORY_SIZE = 100
    MOV_HISTORY_RELEVANCE_T = -.1  # sec; < 0
    AXES = 3

    def __init__(self, do_calc_vel: bool = False, do_calc_acc: bool = False,
                 mov_hist_size: int = MOV_HISTORY_SIZE, mov_history_relevance_t: float = MOV_HISTORY_RELEVANCE_T):
        self.do_calc_vel = do_calc_vel
        self.do_calc_acc = do_calc_acc

        # poll these to get curr val
        self.pos = np.zeros(PlayerMovement.AXES)
        self.vel = np.zeros(PlayerMovement.AXES)
        self.acc = np.zeros(PlayerMovement.AXES)

        def _smoothing(rel_ts):  # linear windowing of previous pos recordings to compute vel or acc
            w = np.clip(rel_ts - mov_history_relevance_t, 0, None)
            return w / w.sum()  # TODO test if this smoothing is as good as the one implemented in motion sensing

        self.calc_some = do_calc_vel is None or do_calc_acc is None
        self.smoothing = _smoothing
        self._ts = deque(maxlen=mov_hist_size)  # time points and..
        self._ps = deque(maxlen=mov_hist_size)  # positions to calc vel/acc

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
