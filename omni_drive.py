import sys
import time
import signal
import sched
import numpy as np
import RPi.GPIO as GPIO

from actuator import LinActuator
from motion import get_front_motion_sensor, get_side_motion_sensor


class OmniDrive:
    def __init__(self, roller_pins, lin_act_pins, up_trans_t=6, down_trans_t=5, pwm_freq=1000):

        self.roller_dirs = np.array(['left', 'right'])
        self.simple_dirs = {'forward': [1., 0., 0.], 'backward': [-1., 0., 0.],  # v, vn, w
                            'left_strafe': [0., -1., 0.], 'right_strafe': [0., 1., 0.],
                            'left_turn': [0., 0., -1.], 'right_turn': [0., 0., 1.]}
        self.simple_dirs = {k: np.array(v) for k, v in self.simple_dirs.items()}

        self.d = (10. + 6.75) / 100.  # m, wheel distance
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

    def trapesoid_accelerate(self):
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

    def calibrate_direction(self):
        flo = get_front_motion_sensor()

        # TODO capture dominant axis / not dominant axis ratio; try to minimize it by trying different d values
        #   first then add the pi/3 as an extra opt param if necessary;
        #   have the d values and corresponding ratios printed in an array format, so they can be analyzed in
        #   matplotlib locally with copy-pasting - later just use argmax, only for testing now have the plotting

        # TODO use the other motion sensor too: get_side_motion_sensor

        # TODO after param optimization is done, set lvl of ignorance to avoid noise


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
