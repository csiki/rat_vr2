import sys
import time
import signal
import sched
import numpy as np
import RPi.GPIO as GPIO

from actuator import LinActuator
from motion import get_front_motion_sensor, get_side_motion_sensor, MotionSensor


class OmniDrive:
    def __init__(self, roller_pins, lin_act_pins, up_trans_t=6, down_trans_t=5, pwm_freq=1000):

        self.roller_dirs = np.array(['left', 'right'])
        self.simple_dirs = {'forward': [1., 0., 0.], 'backward': [-1., 0., 0.],  # v, vn, w
                            'left_strafe': [0., -1., 0.], 'right_strafe': [0., 1., 0.],
                            'left_turn': [0., 0., -1.], 'right_turn': [0., 0., 1.]}
        self.simple_dirs = {k: np.array(v) for k, v in self.simple_dirs.items()}

        self.def_d = (10. + 6.75) / 100.  # m, wheel distance
        self.d = self.def_d
        self.noise_floors = np.zeros(3)  # 3 axes
        # self.trans_mx = self._get_trans_mx()

        self.roller_pins = roller_pins

        self.up_trans_t = up_trans_t
        self.down_trans_t = down_trans_t
        self.pwm_freq = pwm_freq

        self.pwms = None
        self.scheduler = sched.scheduler(time.time, time.sleep)
        self.driving = False

        self.lin_act = LinActuator(lin_act_pins)

        GPIO.setmode(GPIO.BCM)

    def _get_trans_mx(self):
        return np.array(
            [[-np.sin(np.pi / 3), np.cos(np.pi / 3), self.d],
             [0, -1, self.d],
             [np.sin(np.pi / 3), np.cos(np.pi / 3), self.d]]
        )

    def setup(self):
        self.pwms = []

        for pins in self.roller_pins:
            for n in pins.values():
                GPIO.setup(n, GPIO.OUT, initial=GPIO.LOW)

            # pwms
            if 'pwm' in pins:
                pwm = GPIO.PWM(pins['pwm'], self.pwm_freq)
                self.pwms.append(pwm)

        self.lin_act.setup()
        print('OmniDrive setup done')

    def mount(self, blocking=False, callback=None):
        self.lin_act.drive('down', self.down_trans_t, blocking=blocking)
        if callback is not None:
            self.scheduler.enter(self.down_trans_t, 2, callback)

    def letgo(self, blocking=False, callback=None):
        self.lin_act.drive('up', self.up_trans_t, blocking=blocking)
        if callback is not None:
            self.scheduler.enter(self.up_trans_t, 2, callback)

    def calc_wheel_v(self, drive_v):
        highest_abs_v = np.abs(drive_v).max()
        if highest_abs_v > 1.:
            print(f'Highest absolute velocity cannot be over 1.0: {highest_abs_v}!', file=sys.stderr)

        drive_v[0] = -1 * drive_v[0]  # flip forward-backward

        wheel_v = np.matmul(self._get_trans_mx(), drive_v.transpose())
        wheel_v = wheel_v / np.abs(wheel_v).max() * highest_abs_v  # normalize then scale back to requested velocity

        wheel_dc = np.abs(wheel_v * 100.)  # duty cycle max 100
        wheel_dir = self.roller_dirs[(wheel_v > 0).astype(int)]

        return wheel_dir, wheel_dc

    def drive(self, wheel_dir, wheel_dc, t=None, blocking=False, callback=None):
        # set duty cycle
        for i, pins in enumerate(self.roller_pins):
            GPIO.output(pins['left'], GPIO.LOW)
            GPIO.output(pins['right'], GPIO.LOW)
            self.pwms[i].start(wheel_dc[i])

        # start
        for i, pins in enumerate(self.roller_pins):
            GPIO.output(pins[wheel_dir[i]], GPIO.HIGH)

        self.driving = True

        # stop
        if t is not None and blocking:
            time.sleep(t)
            self.stop()
        elif t is not None and not blocking:
            self.scheduler.enter(t, 1, self.stop)

        if t is not None and callback is not None:
            self.scheduler.enter(t, 2, callback)

    def simple_drive(self, simple_dir: str, t=None, blocking=False, callback=None):
        if simple_dir not in self.simple_dirs:
            print(f'Direction {simple_dir} is not any of: {", ".join(self.simple_dirs)}!', file=sys.stderr)
            return
        print(f'Drive {simple_dir} for {t} seconds')

        drive_v = self.simple_dirs[simple_dir]
        wheel_dir, wheel_dc = self.calc_wheel_v(drive_v)
        self.drive(wheel_dir, wheel_dc, t, blocking, callback)

    def trapesoid_accelerate(self):  # TODO solve it in the loop()
        pass  # TODO

    def stop(self):
        for i, pins in enumerate(self.roller_pins):
            GPIO.output(pins['left'], GPIO.LOW)
            GPIO.output(pins['right'], GPIO.LOW)
            self.pwms[i].stop()

        self.driving = False

    def loop(self):
        self.scheduler.run(blocking=False)
        self.lin_act.loop()

    def cleanup(self):
        self.stop()
        self.lin_act.cleanup()
        GPIO.cleanup()
        print('OmniDrive cleanup done')

    def calibrate_direction(self):  # TODO test this
        flo = MotionSensor(0, 'front')

        rng = self.def_d * .5
        ds = np.linspace(self.def_d - rng, self.def_d + rng, 10)
        dirs_to_try = ['forward', 'backward', 'turn_left', 'turn_right']  # TODO try strafe l/r w/ other motion sensor
        front_motion_dir_dominance = [self.simple_dirs[dir_].abs()[[0, 2]] for dir_ in dirs_to_try]
        dir_motion_matches = [[] for _ in dirs_to_try]  # inner list ordered as ds
        axis_noise_floors = [[] for _ in range(2)]  # inner list ordered as motion sensed axes

        for d in ds:
            self.d = d

            for dir_i, (dir_, dom) in enumerate(zip(dirs_to_try, front_motion_dir_dominance)):
                self.simple_drive(dir_, t=5, blocking=False)

                try:
                    while self.driving:
                        flo.loop()
                        self.loop()
                        time.sleep(0.1)
                    flo.loop()

                    motion = flo.get_rel_motion()
                    motion_match = (motion * dom/dom.sum()) / motion.sum()
                    dir_motion_matches[dir_i].append(motion_match)

                    noise_floor = motion[dom == 0].sum()
                    for axis_i in np.where(dom == 0)[0]:
                        axis_noise_floors[axis_i].append(noise_floor)

                except KeyboardInterrupt:
                    self.stop()

        mean_motion_matches = np.array(dir_motion_matches).mean(axis=0)  # mean across directions

        # TODO rm, only for local matplotlib visualization and testing
        print('Diameters:', ds.tolist())
        print('Mean motion matches:', mean_motion_matches.tolist())
        print('Directions:', dirs_to_try)
        print('All motion matches:', dir_motion_matches)

        # select diameter with the best performance
        self.d = ds[np.argmax(mean_motion_matches)]
        print(f'Direction calibration done; best diameter selected: {self.d:.03f}')

        # set lvl of ignorance to avoid noise; only look at noise at the selected best diameter for each axis
        mean_noise_floors = np.array(axis_noise_floors).mean(axis=1)  # per axis
        self.noise_floors[[0, 2]] = mean_noise_floors  # TODO compute it also for strafe axis with other sensor

        # TODO actually do something with noise floors


    def calibrate_full_turn(self):
        pass  # TODO

    def calibrate_acc(self):
        pass  # TODO


def wheel_tests(omni_drive: OmniDrive):
    # straight test
    print('straight test')
    drive_v = np.array([1., 0., 0.])  # v, vn, w

    wheel_dir, wheel_dc = omni_drive.calc_wheel_v(drive_v)
    print(wheel_dir, wheel_dc)
    omni_drive.drive(wheel_dir, wheel_dc, 5, blocking=True)
    time.sleep(2)

    # left strafe test
    print('left strafe test')
    drive_v = np.array([0., -1., 0.])  # v, vn, w

    wheel_dir, wheel_dc = omni_drive.calc_wheel_v(drive_v)
    print(wheel_dir, wheel_dc)
    omni_drive.drive(wheel_dir, wheel_dc, 5, blocking=True)
    time.sleep(2)

    # right strafe test
    print('right strafe test')
    drive_v = np.array([0., 1., 0.])  # v, vn, w

    wheel_dir, wheel_dc = omni_drive.calc_wheel_v(drive_v)
    print(wheel_dir, wheel_dc)
    omni_drive.drive(wheel_dir, wheel_dc, 5, blocking=True)
    time.sleep(2)

    # counter-clockwise turn test
    print('counter-clockwise = left turn test')
    drive_v = np.array([0., 0., -1.])  # v, vn, w

    wheel_dir, wheel_dc = omni_drive.calc_wheel_v(drive_v)
    print(wheel_dir, wheel_dc)
    omni_drive.drive(wheel_dir, wheel_dc, 5, blocking=True)
    time.sleep(2)

    # clockwise turn test
    print('clockwise = right turn test')
    drive_v = np.array([0., 0., 1.])  # v, vn, w

    wheel_dir, wheel_dc = omni_drive.calc_wheel_v(drive_v)
    print(wheel_dir, wheel_dc)
    omni_drive.drive(wheel_dir, wheel_dc, 5, blocking=True)
    time.sleep(2)


def _main():
    lin_act_pins = {'up': 22, 'down': 4, 'enable': 27}
    roller0_pins = {'right': 23, 'left': 24, 'pwm': 18}
    roller1_pins = {'right': 5, 'left': 6, 'pwm': 13}
    roller2_pins = {'right': 25, 'left': 26, 'pwm': 12}
    roller_pins = [roller0_pins, roller1_pins, roller2_pins]

    omni_drive = OmniDrive(roller_pins, lin_act_pins, up_trans_t=6, down_trans_t=6, pwm_freq=1000)
    omni_drive.setup()

    def exit_code(*args):
        omni_drive.cleanup()
        exit(0)

    signal.signal(signal.SIGINT, exit_code)

    wheel_tests(omni_drive)


if __name__ == '__main__':
    _main()
