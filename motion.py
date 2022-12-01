import time
import argparse
import numpy as np
import importlib
from typing import Callable, Any, Union, Tuple, List
from collections import deque
from importlib.util import find_spec

if find_spec('pmw3901'):
    from pmw3901 import PMW3901, PAA5100
else:  # testing
    PMW3901 = PAA5100 = None
    print('pmw3901 NOT FOUND')


class MotionSensor:  # on-ball movement tracking
    def __init__(self, spi_port, spi_cs, spi_slot, sensor_class=PAA5100, invert_x=False, invert_y=False, swap_xy=False):
        # spi_port=0, spi_slot='front' for the front sensor
        spi_cs_gpio = [7, 16]  # spi0 and spi1

        self.sensor = sensor_class(spi_port=spi_port, spi_cs=spi_cs, spi_cs_gpio=spi_cs_gpio[spi_slot])
        self.sensor.set_orientation(invert_x=invert_x, invert_y=invert_y, swap_xy=swap_xy)

        self._last_rel_t, self._last_rec = time.time(), 0
        self.rel_x, self.rel_y = 0, 0
        self.abs_x, self.abs_y = 0, 0
        self._reset_rel_next_iter = False

    def loop(self):
        # reset relative motion if get_rel_motion() was called after the previous loop() run
        if self._reset_rel_next_iter:
            self.rel_x, self.rel_y = 0, 0
            self._last_rel_t = self._last_rec
            self._reset_rel_next_iter = False

        # store motion information
        try:
            x, y = self.sensor.get_motion(timeout=0.001)
            self._last_rec = time.time()  # FIXME moved it here; is this fine? still not accurate enough - maybe the limitation of the sensor
            self.rel_x += x
            self.rel_y += y
            self.abs_x += x
            self.abs_y += y
        except RuntimeError as e:
            pass  # timed out waiting for motion data

    def get_rel_motion(self, get_dt=False):
        rel_mot = np.array([self.rel_x, self.rel_y], dtype=np.float32)
        self._reset_rel_next_iter = True

        dt = np.float32(self._last_rec - self._last_rel_t)
        # self._last_rel_t = time.time()

        if get_dt:
            return rel_mot, dt
        return rel_mot

    def get_abs_motion(self):
        return np.array([self.abs_x, self.abs_y], dtype=np.float32)

    def get_vel(self):
        rel_mot, dt = self.get_rel_motion(get_dt=True)
        return rel_mot / dt


class MotionSensors:  # 3 degrees of freedom

    # front, then side sensor mapping; first int is motion axis index of a sensor, second is the 3 DoF axis index
    DEFAULT_AXIS_MAPPING = [((0, 0), (1, 2)), ((0, 1), (1, 2))]

    def __init__(self, front_flo: MotionSensor, side_flo: MotionSensor,
                 axis_mapping: List[Tuple[Tuple[int, int], Tuple[int, int]]] = DEFAULT_AXIS_MAPPING):
        self.front_flo = front_flo
        self.side_flo = side_flo
        self.axis_mapping = axis_mapping

    def loop(self):
        self.front_flo.loop()
        self.side_flo.loop()

    def _combine_sensor_rec(self, recs, get_dt):
        # axis mapping be like [((0, 0), (1, 1)), ((0, 1), (1, 2))]
        dts = [r[1] for r in recs if get_dt]
        recs = [(r[0] if get_dt else r) for r in recs]

        xyz = [[], [], []]  # gather motion recording for each axis, then mean when sensors have axis overlap
        for rec, amap in zip(recs, self.axis_mapping):
            for _from, _to in amap:
                xyz[_to].append(rec[_from])

        combined = np.array([np.mean(d) for d in xyz], dtype=np.float32)
        if get_dt:
            return combined, np.mean(dts)
        return combined

    def get_rel_motion(self, get_dt=False):
        rel_mots = [self.front_flo.get_rel_motion(get_dt), self.side_flo.get_rel_motion(get_dt)]
        xyz = self._combine_sensor_rec(rel_mots, get_dt)
        return xyz

    def get_abs_motion(self):
        abs_mots = [self.front_flo.get_abs_motion(), self.side_flo.get_abs_motion()]
        xyz = self._combine_sensor_rec(abs_mots, get_dt=False)
        return xyz

    def get_vel(self):
        vels = [self.front_flo.get_vel(), self.side_flo.get_vel()]
        xyz = self._combine_sensor_rec(vels, get_dt=False)
        return xyz


class SmoothMotion:  # MotionSensor wrapper
    def __init__(self, flo: Union[MotionSensor, MotionSensors], smooth_dt):
        self.flo = flo
        self.smooth_dt = smooth_dt

        self._rel_mots = deque()
        self._dts = deque()

        def _smoothing(dts):
            # windowing of previous pos recordings to compute vel or acc
            # in the time window given, weights recordings with longer dt higher; same as in player movement
            past = np.cumsum(np.abs(dts[::-1]))[::-1]  # 10, 7, 5, 2, 1
            within_t = past - smooth_dt  # smooth_dt = 6; 4, 1, -1, -4, -5
            t_occupied = np.clip(dts - within_t, 0, None)  # 0, 1, 4, 5, 6
            t_occupied = np.min([t_occupied, dts], axis=0)  # 0, 1, 3, 1, 1
            weight = t_occupied / t_occupied.sum()  # normalize
            return weight

        self.smoothing = _smoothing

    def loop(self):
        self.flo.loop()
        # record motion at each loop
        rel_mot, dt = self.flo.get_rel_motion(get_dt=True)
        self._rel_mots.append(rel_mot)
        self._dts.append(dt)

    def get_vel(self):
        # computes velocity over time by weighting velocities in the given past window by their dt,
        # that is, velocities that were present for longer are weighted higher in the mean
        if sum(self._dts, 0.) < self.smooth_dt:
            return np.zeros(3)

        rel_mots = np.asarray(self._rel_mots)  # (time, axes)
        dts = np.asarray(self._dts)  # example: 3, 2, 3, 1, 1
        # print(dts.shape)

        # in the time window given, weights recordings with longer dt higher
        # print(np.concatenate([dts[..., None], rel_mots], axis=1))
        smooth_rel_mot = (rel_mots * self.smoothing(dts)[:, None]).sum(axis=0)

        while np.abs(self._dts).sum() > self.smooth_dt * 4:  # have some cushion
            self._rel_mots.popleft()
            self._dts.popleft()

        smooth_rel_mot[np.isnan(smooth_rel_mot)] = 0.
        return smooth_rel_mot.astype(np.float32)


def _main():  # example code with OmniDrive

    from omni_drive import OmniDrive

    flo = MotionSensor(0, 1, 0)

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
