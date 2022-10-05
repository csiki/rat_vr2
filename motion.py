import time
import argparse
import numpy as np
from pmw3901 import PMW3901, PAA5100, BG_CS_FRONT_BCM, BG_CS_BACK_BCM
from typing import Callable, Any, Union, Tuple, List
from collections import deque


class MotionSensor:
    def __init__(self, spi_port, spi_slot, sensor_class=PAA5100):
        # spi_port=0, spi_slot='front' for the front sensor
        self.sensor = sensor_class(spi_port=spi_port, spi_cs=1,
                                   spi_cs_gpio=BG_CS_FRONT_BCM if spi_slot == 'front' else BG_CS_BACK_BCM)
        self.sensor.set_orientation(invert_x=True, invert_y=False, swap_xy=False)

        self._last_rel_t, self._last_rec = time.time(), 0
        self.rel_x, self.rel_y = 0, 0
        self.abs_x, self.abs_y = 0, 0
        self._reset_rel_next_iter = False

    def loop(self):
        # reset relative motion if get_rel_motion() was called after the previous loop() run
        if self._reset_rel_next_iter:
            self.rel_x, self.rel_y = 0, 0
            self._reset_rel_next_iter = False

        # store motion information
        try:
            x, y = self.sensor.get_motion(timeout=0.001)
            self._last_rec = time.time()
            self.rel_x += x
            self.rel_y += y
            self.abs_x += x
            self.abs_y += y
        except RuntimeError:
            pass

    def get_rel_motion(self, get_dt=False):
        rel_mot = np.array([self.rel_x, self.rel_y])
        self._reset_rel_next_iter = True

        dt = self._last_rec - self._last_rel_t
        self._last_rel_t = time.time()

        if get_dt:
            return rel_mot, dt
        return rel_mot

    def get_abs_motion(self):
        return np.array([self.abs_x, self.abs_y])

    def get_vel(self, reset=True):
        rel_mot, dt = self.get_rel_motion(get_dt=True)
        return rel_mot / dt


class MotionSensors:  # 3 degrees of freedom

    # front, then side sensor mapping; first int is motion axis index of a sensor, second is the 3 DoF axis index
    DEFAULT_AXIS_MAPPING = [((0, 0), (1, 2)), ((0, 1), (1, 2))]  # TODO test

    def __init__(self, front_flo: MotionSensor, side_flo: MotionSensor,
                 axis_mapping: List[Tuple[Tuple[int, int], Tuple[int, int]]] = DEFAULT_AXIS_MAPPING):
        self.front_flo = front_flo
        self.side_flo = side_flo
        self.axis_mapping = axis_mapping

    def loop(self):
        self.front_flo.loop()
        self.side_flo.loop()

    def _combine_sensor_rec(self, recs):
        # axis mapping be like [((0, 0), (1, 1)), ((0, 1), (1, 2))]
        xyz = [[], [], []]  # gather motion recording for each axis, then mean when sensors have axis overlap
        for rec, amap in zip(recs, self.axis_mapping):
            for _from, _to in amap:
                xyz[_to].append(rec[_from])
        return np.mean(xyz, axis=1)

    def get_rel_motion(self, get_dt=False):
        rel_mots = [self.front_flo.get_rel_motion(get_dt), self.side_flo.get_rel_motion(get_dt)]
        xyz = self._combine_sensor_rec(rel_mots)
        return xyz

    def get_abs_motion(self):
        abs_mots = [self.front_flo.get_abs_motion(), self.side_flo.get_abs_motion()]
        xyz = self._combine_sensor_rec(abs_mots)
        return xyz

    def get_vel(self):
        vels = [self.front_flo.get_vel(), self.side_flo.get_vel()]
        xyz = self._combine_sensor_rec(vels)
        return xyz


class SmoothMotion:  # MotionSensor wrapper
    def __init__(self, flo: Union[MotionSensor, MotionSensors], smooth_dt):
        self.flo = flo
        self.smooth_dt = smooth_dt

        self._rel_mots = deque()
        self._dts = deque()

    def get_vel(self):
        # computes velocity over time by weighting velocities in the given past window by their dt,
        # that is, velocities that were present for longer are weighted higher in the mean
        rel_mot, dt = self.flo.get_rel_motion(get_dt=True)
        self._rel_mots.append(rel_mot)
        self._dts.append(dt)

        rel_mots = np.asarray(self._rel_mots)
        dts = np.asarray(self._dts)  # example: 3, 2, 3, 1, 1

        # this might be overcomplicated, who knows
        past = np.cumsum(dts[::-1])  # 10, 7, 5, 2, 1
        within_t = past - self.smooth_dt  # smooth_dt = 6; 4, 1, -1, -4, -5
        t_occupied = np.clip(dts - within_t, 0, None)  # 0, 1, 4, 5, 6
        t_occupied = np.min([t_occupied, dts], axis=1)  # 0, 1, 3, 1, 1
        weight = t_occupied / t_occupied.sum()  # normalize

        smooth_rel_mot = (rel_mots * weight).sum()

        while sum(self._dts) > self.smooth_dt * 4:  # have some cushion
            self._rel_mots.popleft()
            self._dts.popleft()

        return smooth_rel_mot


def _main():  # example code with OmniDrive

    from omni_drive import OmniDrive

    flo = MotionSensor(0, 'front')

    lin_act_pins = {'up': 22, 'down': 4, 'enable': 27}
    roller0_pins = {'right': 23, 'left': 24, 'pwm': 18}
    roller1_pins = {'right': 5, 'left': 6, 'pwm': 13}
    roller2_pins = {'right': 25, 'left': 26, 'pwm': 12}
    roller_pins = [roller0_pins, roller1_pins, roller2_pins]

    omni_drive = OmniDrive(roller_pins, lin_act_pins, up_trans_t=6, down_trans_t=6, pwm_freq=1000)
    omni_drive.setup()
    omni_drive.simple_drive('forward', t=10, blocking=False)

    cx = 0
    cy = 0
    c = 0
    tprev = 0

    try:
        while True:
            try:
                omni_drive.loop()
                flo.loop()
            except RuntimeError:
                continue

            tx, ty = flo.get_abs_motion()

            if c >= 150:
                cx, cy = flo.get_rel_motion()
                # print("Relative: cx {:03d} cy {:03d} | Absolute: x {:03d} y {:03d}".format(cx, cy, tx, ty))
                cx = cy = c = 0

            tnow = int(time.time())
            if tnow >= tprev + 1:
                print("{}->{}: relative: cx {:03d} cy {:03d} | Absolute: x {:03d} y {:03d}".format(tprev, tnow, cx, cy,
                                                                                                   tx, ty))
                tprev = tnow

            c += 1
            time.sleep(0.1)

    except KeyboardInterrupt:
        omni_drive.cleanup()


if __name__ == '__main__':
    _main()
