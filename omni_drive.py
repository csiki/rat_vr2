import sys
import time
import signal
import sched
import numpy as np
import RPi.GPIO as GPIO
import torch

from actuator import LinActuator
from motion import MotionSensor


class OmniDrive:
    def __init__(self, roller_pins, lin_act_pins, up_trans_t=6, down_trans_t=5, pwm_freq=1000):

        self.roller_dirs = np.array(['left', 'right'])
        self.simple_dirs = {'forward': [1., 0., 0.], 'backward': [-1., 0., 0.],  # v, vn, w
                            'left_strafe': [0., -1., 0.], 'right_strafe': [0., 1., 0.],
                            'left_turn': [0., 0., -1.], 'right_turn': [0., 0., 1.]}
        self.simple_dirs = {k: np.array(v) for k, v in self.simple_dirs.items()}

        self.def_d = (10. + 6.75) / 100.  # m, wheel distance
        self.d = self.def_d
        self.noise_floors = np.zeros(3)  # noise/second; 3 axes
        self.trans_mx = np.array(
            [[-np.sin(np.pi / 3), np.cos(np.pi / 3), self.d],
             [0, -1, self.d],
             [np.sin(np.pi / 3), np.cos(np.pi / 3), self.d]]
        )

        self.roller_pins = roller_pins

        self.up_trans_t = up_trans_t
        self.down_trans_t = down_trans_t
        self.pwm_freq = pwm_freq

        self.pwms = None
        self.scheduler = sched.scheduler(time.time, time.sleep)
        self.driving = False

        self.lin_act = LinActuator(lin_act_pins)

        GPIO.setmode(GPIO.BCM)

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

        wheel_v = np.matmul(self.trans_mx, drive_v.transpose())
        wheel_v_normed = wheel_v / np.abs(wheel_v).max() * highest_abs_v  # normalize then scale back to requested velocity

        wheel_dc = np.abs(wheel_v_normed * 100.)  # duty cycle max 100
        wheel_dir = self.roller_dirs[(wheel_v_normed > 0).astype(int)]

        return wheel_dir, wheel_dc, wheel_v, wheel_v_normed

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
        wheel_dir, wheel_dc, _, _ = self.calc_wheel_v(drive_v)
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

    # TODO need to calibrate whether the direction is right: forward vs backward, right vs left
    #   this one calibrates hyperparams of the trans_mx --> rename this and make a calibrate direction function
    # TODO LATER: also calibrate pi/3, not just d... USE BAYESIAN OPT: use the BayesianOptimization package
    def calibrate_transfer_fun(self):  # TODO test this
        flo = MotionSensor(0, 'front')

        rng = self.def_d * .5
        drive_t = 5
        ds = np.linspace(self.def_d - rng, self.def_d + rng, 10)
        dirs_to_try = ['forward', 'backward', 'left_turn', 'right_turn']  # TODO try strafe l/r w/ other motion sensor

        front_motion_dir_dominance = [np.abs(self.simple_dirs[dir_][[0, 2]]) for dir_ in dirs_to_try]
        dir_motion_matches = [[] for _ in dirs_to_try]  # inner list ordered as ds
        axis_noise_floors = [[] for _ in range(2)]  # inner list ordered as motion sensed axes
        
        print('Number of diameters to try:', len(ds))
        
        for di, d in enumerate(ds):
            self.d = d
            print(f'{di}: d = {d:.3f}')

            for dir_i, (dir_, dom) in enumerate(zip(dirs_to_try, front_motion_dir_dominance)):
                wheel_v = self.simple_drive(dir_, t=drive_t, blocking=False)

                try:
                    while self.driving:
                        flo.loop()
                        self.loop()
                        time.sleep(0.1)
                    flo.loop()

                    # motion abs: assumes that the camera is rotated right: no diff here made of forward vs backward
                    motion = np.abs(flo.get_rel_motion())
                    motion_match = ((motion * (dom/dom.sum())) / motion.sum()).sum()
                    dir_motion_matches[dir_i].append(motion_match.tolist())

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
        print(f'Direction calibration done; best diameter selected: {self.d:.03f}; default was {self.def_d:.03f}')

        # set lvl of ignorance to avoid noise; only look at noise at the selected best diameter for each axis
        mean_noise_floors = np.array(axis_noise_floors).mean(axis=1)  # per axis
        self.noise_floors[[0, 2]] = mean_noise_floors / drive_t  # TODO compute it also for strafe axis with other sensor
        print('Noise floor:', self.noise_floors.tolist())

        # TODO actually do something with noise floors

    def calibrate_transfer_fun2(self):
        import omni_transfer

        flo = MotionSensor(0, 'front')
        drive_t = 5
        niter = 50
        dirs_to_try = ['forward', 'backward', 'left_turn', 'right_turn']  # TODO try strafe l/r w/ other motion sensor
        desired_motion = [self.simple_dirs[dir_] for dir_ in dirs_to_try]  # not abs
        # desired_motion = torch.reshape(torch.tensor(desired_motion), (len(dirs_to_try), 3, 1))

        # setup nn
        trans = omni_transfer.OmniTransfer(self.def_d)
        loss = torch.nn.MSELoss()
        opt = torch.optim.SGD(trans.parameters(), lr=0.05)
        errs = []
        trans_mxs = []

        print('Transfer function calibration begins..')
        for it in range(niter):
            print(f'{it}.', '-' * 10)

            # record actual motion
            actual_motion = []
            wheel_vs = []

            for dir_i, dir_ in enumerate(dirs_to_try):
                drive_v = self.simple_dirs[dir_]
                wheel_dir, wheel_dc, wheel_v, wheel_v_normed = self.calc_wheel_v(drive_v)
                self.drive(wheel_dir, wheel_dc, t=drive_t, blocking=False)
                wheel_vs.append(wheel_v)

                try:
                    while self.driving:
                        flo.loop()
                        self.loop()
                        time.sleep(0.1)
                    flo.loop()

                    motion = flo.get_rel_motion()
                    motion = motion / np.abs(motion).sum()
                    motion = [motion[0], 0, motion[1]]  # f/b and w, TODO no strafe yet
                    actual_motion.append(motion)

                except KeyboardInterrupt:
                    self.stop()
                    return

            # evaluate model for expected motion
            actual_motion = torch.reshape(torch.tensor(actual_motion), (len(dirs_to_try), 3, 1))
            wheel_vs = torch.reshape(torch.tensor(np.array(wheel_vs)), (len(dirs_to_try), 3, 1))
            expected_motion = trans(wheel_vs)
            err = loss(expected_motion, actual_motion)
            print(f'loss: {err.item():.3f}')
            errs.append(err.item())

            opt.zero_grad()
            err.backward()
            opt.step()

            # update transfer mx
            with torch.no_grad():
                updated_trans_mx = torch.inverse(trans.inv_tm_v.data.clone()).numpy()
            trans_mxs.append(updated_trans_mx)
            print('delta transfer:\n', updated_trans_mx - self.trans_mx)
            self.trans_mx = updated_trans_mx

        # choose the best transfer mx
        self.trans_mx = trans_mxs[np.argmin(errs)]
        print(f'Lowest error: {min(errs):.3f}')
        print('Corresponding transition mx:\n', self.trans_mx)

        print('Testing now..')
        for dir_ in dirs_to_try:
            self.simple_drive(dir_, t=drive_t, blocking=True)

    def calibrate_full_turn(self):
        pass  # TODO

    def calibrate_acc(self):
        pass  # TODO


def wheel_tests(omni_drive: OmniDrive):
    # straight test
    print('straight test')
    drive_v = np.array([1., 0., 0.])  # v, vn, w

    wheel_dir, wheel_dc, _, _ = omni_drive.calc_wheel_v(drive_v)
    print(wheel_dir, wheel_dc)
    omni_drive.drive(wheel_dir, wheel_dc, 5, blocking=True)
    time.sleep(2)

    # left strafe test
    print('left strafe test')
    drive_v = np.array([0., -1., 0.])  # v, vn, w

    wheel_dir, wheel_dc, _, _ = omni_drive.calc_wheel_v(drive_v)
    print(wheel_dir, wheel_dc)
    omni_drive.drive(wheel_dir, wheel_dc, 5, blocking=True)
    time.sleep(2)

    # right strafe test
    print('right strafe test')
    drive_v = np.array([0., 1., 0.])  # v, vn, w

    wheel_dir, wheel_dc, _, _ = omni_drive.calc_wheel_v(drive_v)
    print(wheel_dir, wheel_dc)
    omni_drive.drive(wheel_dir, wheel_dc, 5, blocking=True)
    time.sleep(2)

    # counter-clockwise turn test
    print('counter-clockwise = left turn test')
    drive_v = np.array([0., 0., -1.])  # v, vn, w

    wheel_dir, wheel_dc, _, _ = omni_drive.calc_wheel_v(drive_v)
    print(wheel_dir, wheel_dc)
    omni_drive.drive(wheel_dir, wheel_dc, 5, blocking=True)
    time.sleep(2)

    # clockwise turn test
    print('clockwise = right turn test')
    drive_v = np.array([0., 0., 1.])  # v, vn, w

    wheel_dir, wheel_dc, _, _ = omni_drive.calc_wheel_v(drive_v)
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

    omni_drive.calibrate_transfer_fun2()

    # tests
    # wheel_tests(omni_drive)

    omni_drive.cleanup()


if __name__ == '__main__':
    _main()
