import time
import signal
import sched
import numpy as np
import RPi.GPIO as GPIO


class OmniDrive:
    def __init__(self, roller_pins, lin_act_pins, up_trans_t=6, down_trans_t=5, pwm_freq=1000):

        self.roller_dirs = np.array(['left', 'right'])
        self.d = (10. + 6.75) / 100.  # m, wheel distance
        self.trans_mx = np.array(
            [[-np.sin(np.pi / 3), np.cos(np.pi / 3), self.d],
             [0, -1, self.d],
             [np.sin(np.pi / 3), np.cos(np.pi / 3), self.d]]
        )

        self.roller_pins = roller_pins
        self.lin_act_pins = lin_act_pins

        self.up_trans_t = up_trans_t
        self.down_trans_t = down_trans_t
        self.pwm_freq = pwm_freq
        self.pwms = None
        self.scheduler = sched.scheduler(time.time, time.sleep)

        GPIO.setmode(GPIO.BCM)

    def setup(self):
        pin_sets = [self.lin_act_pins, *self.roller_pins]
        self.pwms = []

        for pins in pin_sets:
            for n in pins.values():
                GPIO.setup(n, GPIO.OUT, initial=GPIO.LOW)

            # pwms
            if 'pwm' in pins:
                pwm = GPIO.PWM(pins['pwm'], self.pwm_freq)
                self.pwms.append(pwm)

        print('OmniDrive setup done')

    def calc_wheel_v(self, drive_v):
        drive_v[0] = -1 * drive_v[0]  # flip forward-backward
        wheel_v = np.matmul(self.trans_mx, drive_v.transpose())
        wheel_v = wheel_v / np.abs(wheel_v).max()

        wheel_dc = np.abs(wheel_v * 100.)  # duty cycle max 100
        wheel_dir = self.roller_dirs[(wheel_v > 0).astype(int)]

        return wheel_dir, wheel_dc

    def drive(self, wheel_dir, wheel_dc, t=None, blocking=False):
        # set duty cycle
        for i, pins in enumerate(self.roller_pins):
            GPIO.output(pins['left'], GPIO.LOW)
            GPIO.output(pins['right'], GPIO.LOW)
            self.pwms[i].start(wheel_dc[i])

        # start
        for i, pins in enumerate(self.roller_pins):
            GPIO.output(pins[wheel_dir[i]], GPIO.HIGH)

        # stop
        if t is not None and blocking:
            time.sleep(t)
            self.stop()
        elif t is not None and not blocking:
            self.scheduler.enter(t, 1, self.stop)

    def stop(self):
        for i, pins in enumerate(self.roller_pins):
            GPIO.output(pins['left'], GPIO.LOW)
            GPIO.output(pins['right'], GPIO.LOW)
            self.pwms[i].stop()

    def cleanup(self):
        self.stop()
        GPIO.cleanup()
        print('OmniDrive cleanup done')


def wheel_tests(omni_drive: OmniDrive):
    # straight test
    print('straight test')
    drive_v = np.array([1., 0., 0.])  # v, vn, w

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

    # left strafe test
    print('left strafe test')
    drive_v = np.array([0., -1., 0.])  # v, vn, w

    wheel_dir, wheel_dc = omni_drive.calc_wheel_v(drive_v)
    print(wheel_dir, wheel_dc)
    omni_drive.drive(wheel_dir, wheel_dc, 5, blocking=True)
    time.sleep(2)

    # counter-clockwise turn test
    print('counter-clockwise turn test')
    drive_v = np.array([0., 0., 1.])  # v, vn, w

    wheel_dir, wheel_dc = omni_drive.calc_wheel_v(drive_v)
    print(wheel_dir, wheel_dc)
    omni_drive.drive(wheel_dir, wheel_dc, 5, blocking=True)
    time.sleep(2)

    # clockwise turn test
    print('clockwise turn test')
    drive_v = np.array([0., 0., -1.])  # v, vn, w

    wheel_dir, wheel_dc = omni_drive.calc_wheel_v(drive_v)
    print(wheel_dir, wheel_dc)
    omni_drive.drive(wheel_dir, wheel_dc, 5, blocking=True)
    time.sleep(2)


def main():
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
    main()
