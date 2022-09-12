import sys
import time
import signal
import sched
import pickle
import numpy as np
import RPi.GPIO as GPIO
from scipy.optimize import curve_fit
from typing import Callable, Any, Union
import simple_pid
from collections import namedtuple

from actuator import LinActuator
from motion import MotionSensor


class OmniDrive:

    # FB_FUN_TYPE =
    PIDp = namedtuple('PIDp', ['Kp', 'Ki', 'Kd'])
    AXES = 3
    PWM_2_MOTION_FUN = lambda x, *p: x * p[0] + p[1]  # uses pwm_to_motion_p params for each axis

    def __init__(self, roller_pins, lin_act_pins, up_trans_t=6, down_trans_t=5, pwm_freq=1000, calib_path='omni_calib.pckl'):

        self.roller_dirs = np.array(['left', 'right'])
        self.simple_dirs_v = {'forward': [1., 0., 0.], 'backward': [-1., 0., 0.],  # v, vn, w
                              'left_strafe': [0., -1., 0.], 'right_strafe': [0., 1., 0.],
                              'left_turn': [0., 0., -1.], 'right_turn': [0., 0., 1.]}
        self.simple_dirs_v = {k: np.array(v) for k, v in self.simple_dirs_v.items()}

        self.noise_floors = np.zeros(OmniDrive.AXES)  # noise/second; 3 axes
        self.pwm_to_motion_p = np.stack([np.ones(3), np.zeros(3)], axis=1)  # lin fun of scaler and bias for each axis

        # pid stuff
        self.pid_p = [OmniDrive.PIDp(0., 0., 0.) for _ in range(OmniDrive.AXES)]  # PID parameters
        self.pid_t = 0.05  # sec; time constant, time between PID updates
        self.pid_err_scalers = np.ones(OmniDrive.AXES)  # scales err according to pwm to delta game pos

        # transfer mx
        self.def_d = (10. + 6.75) / 100.  # in m, wheel distance from center
        self.trans_mx = np.array(
            [[-np.sin(np.pi / 3), np.cos(np.pi / 3), self.def_d],
             [0, -1, self.def_d],
             [np.sin(np.pi / 3), np.cos(np.pi / 3), self.def_d]]
        )

        self.roller_pins = roller_pins
        self.pwm_freq = pwm_freq
        self.pwms = None

        self.up_trans_t = up_trans_t
        self.down_trans_t = down_trans_t

        self.driving = False
        self.rolling = False
        self.roll_fun: Callable[[OmniDrive], Any] = lambda x: None
        self.scheduler = sched.scheduler(time.time, time.sleep)

        self.lin_act = LinActuator(lin_act_pins)

        # update already calibrated vars
        if calib_path:
            with open(calib_path, 'rb') as f:
                calib = pickle.load(f)
            self.noise_floors = calib['noise_floors']
            self.pwm_to_motion_p = calib['pwm_to_motion_p']
            self.pid_p = calib['pid_p']
            self.pid_t = calib['pid_t']
            self.pid_err_scalers = calib['pid_err_scalers']
            self.trans_mx = calib['trans_mx']

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

    def save(self, calib_path):
        calib = {'noise_floors': self.noise_floors, 'pwm_to_motion_p': self.pwm_to_motion_p,
                 'pid_p': self.pid_p, 'pid_t': self.pid_t, 'pid_err_scalers': self.pid_err_scalers,
                 'trans_mx': self.trans_mx}

        with open(calib_path, 'wb') as f:
            pickle.dump(calib, f)

    def mount(self, blocking=False, callback=None):
        self.lin_act.drive('down', self.down_trans_t, blocking=blocking)
        if callback is not None:
            self.scheduler.enter(self.down_trans_t, 2, callback)

    def letgo(self, blocking=False, callback=None):
        self.lin_act.drive('up', self.up_trans_t, blocking=blocking)
        if callback is not None:
            self.scheduler.enter(self.up_trans_t, 2, callback)

    def calc_wheel_v(self, drive_v, ret_wheel_v=False):
        highest_abs_v = np.abs(drive_v).max()
        if highest_abs_v > 1.:
            print(f'Highest absolute velocity cannot be over 1.0: {highest_abs_v}!', file=sys.stderr)

        drive_v[0] = -1 * drive_v[0]  # flip forward-backward

        wheel_v = np.matmul(self.trans_mx, drive_v.transpose())
        wheel_v_normed = wheel_v / np.abs(wheel_v).max() * highest_abs_v  # normalize then scale back to requested velocity

        wheel_dc = np.abs(wheel_v_normed * 100.)  # duty cycle max 100
        wheel_dir = self.roller_dirs[(wheel_v_normed > 0).astype(int)]

        if ret_wheel_v:
            return wheel_dir, wheel_dc, wheel_v, wheel_v_normed
        return wheel_dir, wheel_dc

    def drive(self, wheel_dir, wheel_dc, t=None, blocking=False, callback=None):

        if self.driving or self.rolling:
            print(f'OmniDrive already running (driving={self.driving}, rolling={self.rolling})! '
                  f'Call stop first!', file=sys.stderr)
            return False

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
            try:
                time.sleep(t)
            except KeyboardInterrupt:
                pass
            finally:
                self.stop()
        elif t is not None and not blocking:
            self.scheduler.enter(t, 1, self.stop)

        if t is not None and callback is not None:
            self.scheduler.enter(t, 2, callback)

        return True

    def simple_drive(self, simple_dir: str, t=None, blocking=False, callback=None):
        if simple_dir not in self.simple_dirs_v:
            print(f'Direction {simple_dir} is not any of: {", ".join(self.simple_dirs_v)}!', file=sys.stderr)
            return
        print(f'Drive {simple_dir} for {t} seconds')

        drive_v = self.simple_dirs_v[simple_dir]
        wheel_dir, wheel_dc = self.calc_wheel_v(drive_v)
        return self.drive(wheel_dir, wheel_dc, t, blocking, callback)

    def roll(self, feedback: Callable[[], np.ndarray], eps: float, blocking=False, callback: Callable[[], Any] = None):
        # performs movement on all three axis according to feedback using 1 pid controller per axis
        # if no blocking, main functionality is run in loop(), only the function to run in there is defined here + setup
        # overall dynamics:
        #   drive_v -> motion_v -> in-game_v -> change position in-game -> feedback of delta position -> pid -> drive_v
        # usage:
        #   get from point A to B in map - turn x degrees from current, or go in x, y direction or all of these at once
        #   if eps close stops or can be stopped at any time (when e.g. been rolling too long); use self.rolling
        #   do chain calls with callbacks on top of callbacks (callback as last function argument always)
        #   slippage possible during drive: feedback should return 3 error values, 1 for each axis;
        #     even if it only needs to turn, it could slip off the original position which is then corrected for

        # no need to provide the setpoint, set it to 0, and return -err from feedback()

        # PID tuning: https://www.youtube.com/watch?v=sFOEsA0Irjs, https://www.youtube.com/watch?v=IB1Ir4oCP5k
        #   https://medium.com/@svm161265/when-and-why-to-use-p-pi-pd-and-pid-controller-73729a708bb5
        # calibration process:
        #   increase Kp to reduce rise time; keep increasing it until overshoot happens
        #   increase Ki to reduce rise time too; helps in noisy environments -> set it to a low value
        #   increase Kd to decrease overshoot; can't be used when noise is present -> set it to a low value, maybe == Ki
        # x0=0, x^=10, v(pwm=1)=5 1/s, v(pwn=0.5)=2 1/s;

        if self.driving or self.rolling:
            print(f'OmniDrive already running (driving={self.driving}, rolling={self.rolling})! '
                  f'Call stop first!', file=sys.stderr)
            return False

        # set up PIDs
        # no need for proportional_on_measurement
        pids = [simple_pid.PID(*self.pid_p[axis], sample_time=self.pid_t, output_limits=(-1., 1.), setpoint=0.,
                               error_map=lambda x: x * self.pid_err_scalers[axis])
                for axis in range(OmniDrive.AXES)]

        if blocking:
            neg_err = feedback()

            while np.abs(np.max(neg_err)) < eps or not self.rolling:
                drive_v = np.array([pid(neg_err[axes]) for axes, pid in enumerate(pids)])
                wheel_dir, wheel_dc = self.calc_wheel_v(drive_v)
                self.drive(wheel_dir, wheel_dc)

            self.stop()
            if callback:
                callback()

            return True

        # not blocking
        # set up function to run in loop that returns drive_v
        def _roll_fun():
            neg_err = feedback()  # the negative error, which enables the constant 0 setpoint

            # when done
            if np.abs(np.max(neg_err)) < eps or not self.rolling:
                self.stop()  # self.rolling -> False
                if callback:
                    callback()
                return

            # roll on
            drive_v = np.array([pid(neg_err[axes]) for axes, pid in enumerate(pids)])
            wheel_dir, wheel_dc = self.calc_wheel_v(drive_v)
            self.drive(wheel_dir, wheel_dc)
            return

        self.roll_fun = _roll_fun
        self.rolling = True
        return True

    def trapesoid_accelerate(self):  # TODO solve it in the loop()
        pass  # TODO is this necessary? only if you want to go up to high speeds, otherwise rats can follow immediately

    def stop(self):
        for i, pins in enumerate(self.roller_pins):
            GPIO.output(pins['left'], GPIO.LOW)
            GPIO.output(pins['right'], GPIO.LOW)
            self.pwms[i].stop()

        self.driving = False
        self.rolling = False

    def loop(self):
        self.scheduler.run(blocking=False)
        self.lin_act.loop()

        if self.rolling:
            self.roll_fun(self)

    def cleanup(self):
        self.stop()
        self.lin_act.cleanup()
        GPIO.cleanup()
        print('OmniDrive cleanup done')

    def calibrate_transfer_fun(self):
        import torch
        import omni_transfer_opt

        flo = MotionSensor(0, 'front')
        drive_t = 5
        niter = 50
        dirs_to_try = ['forward', 'backward', 'left_turn', 'right_turn']  # TODO try strafe l/r w/ other motion sensor

        # setup nn
        trans = omni_transfer_opt.OmniTransferOpt(self.def_d)
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
                drive_v = self.simple_dirs_v[dir_]
                wheel_dir, wheel_dc, wheel_v, wheel_v_normed = self.calc_wheel_v(drive_v, ret_wheel_v=True)
                wheel_vs.append(wheel_v)

                flo.get_rel_motion()  # zero out rel vars
                self.drive(wheel_dir, wheel_dc, t=drive_t, blocking=True)

                motion = flo.get_rel_motion()
                motion = motion / np.abs(motion).sum()
                motion = [motion[0], 0, motion[1]]  # f/b and w, TODO 0 -> no strafe yet
                actual_motion.append(motion)

            # evaluate model for expected motion
            actual_motion = torch.reshape(torch.tensor(actual_motion), (len(dirs_to_try), OmniDrive.AXES, 1))
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

    def calibrate_speed(self):
        # try different speeds, compare with motion sensor recording,
        # establish drive velocity (motor pwm) to motion velocity function for each axis
        drive_t = 5  # s
        pwms_to_try = np.linspace(0.1, 1.0, 10)
        dirs_to_try = ['forward', 'backward', 'left_turn', 'right_turn']  # TODO strafe
        flo = MotionSensor(0, 'front')

        drive_vs = []
        motions = []

        for dir_i, dir_ in enumerate(dirs_to_try):
            print('Direction:', dir_)
            for pwm in pwms_to_try:
                print(f'pwm: {pwm}')

                drive_v = self.simple_dirs_v[dir_] * pwm
                wheel_dir, wheel_dc = self.calc_wheel_v(drive_v)
                drive_vs.append(drive_v)

                flo.get_rel_motion()  # zero out rel vars
                self.drive(wheel_dir, wheel_dc, t=drive_t, blocking=True)
                motion = flo.get_rel_motion()
                motion = [motion[0], 0, motion[1]]  # TODO add strafe
                motions.append(motion)

        # establish pwm to motion velocity function for each axis
        #   do scipy function fitting for each axis
        drive_vs = np.abs(np.array(drive_vs))  # abs: direction agnostic within axis
        motions = np.abs(np.array(motions))  # same
        axes = np.argmax(np.abs(drive_vs), axis=1)

        for axis in [0, 2]:  # TODO add strafe and [0, 1, 2]
            dv = drive_vs[axes == axis, axis]  # pwms (see scaling above)
            mv = motions[axes == axis, axis] / drive_t  # motion/sec

            f = lambda x, *p: p[0] * x + p[1]  # TODO test if higher order function is needed
            p0 = np.ones(2)

            popt, pcov = curve_fit(f, dv, mv, p0)
            self.pwm_to_motion_p[axis, :] = popt

            print(f'axis {axis}) popt: {popt.tolist()}')

    def calibrate_full_rot(self):
        pass  # TODO probs needs manual intervention by taking inputs from python (through ssh) (no fucking button)
        # TODO do for all axes separately; the turn axis is the most important: degree to motion conversion
        # TODO omni drive function (see above) that turns a certain degree with pid and the info taken here

    def calibrate_roll_motion(self):
        # TODO
        # Doom player has its sliding acceleration and deceleration (sliding ice-cube) with forces_applied - friction
        #   and forces_applied is defined by the recorded ball motion
        # acceleration and deceleration needs to be incorporated in rolling:
        #   in 1) calculate acceleration, deceleration (same as accel?) and the stable velocity from drive_v
        #   in 2) empirically adjust PID params to avoid overshoot
        # put the player in an arena then call this function
        # 1) drive - game velocity translation
        #    omni driver runs drive() in straight directions xor turns and tracks in-game position (and vel., accel.)
        #    calc drive velocity to game velocity function
        #    derive acceleration, deceleration and stable in-game velocity from drive velocity (if dependent on it)
        # 2) optimize PID params
        #    the omni driver sets goals for itself (in game delta coordinates = fb error)
        #    omni driver runs roll() (either straight x,y motion or turns) to get to the goal,
        #      while tracking in-game motion
        #    optimizes Kp, Ki and Kd for each axis; init them according to translation (vel., accel.) calculated in 1);
        #    change Kp, Ki, Kd up/down until reaching the fast conversion without overshooting
        pass


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

    wheel_dir, wheel_dc  = omni_drive.calc_wheel_v(drive_v)
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

    opt_trans_mx = None
    # opt_trans_mx = [[ 1.16646569,  0.5,         0.1634709 ],
    #                 [-0.00351549, -1.,          0.16910739],
    #                 [-1.17349667,  0.5,         0.17474389]]
    omni_drive = OmniDrive(roller_pins, lin_act_pins, up_trans_t=6, down_trans_t=6, pwm_freq=1000,
                           trans_mx=opt_trans_mx)
    omni_drive.setup()

    def exit_code(*args):
        omni_drive.cleanup()
        exit(0)

    signal.signal(signal.SIGINT, exit_code)

    # omni_drive.calibrate_transfer_fun2()

    # tests
    wheel_tests(omni_drive)

    omni_drive.cleanup()


if __name__ == '__main__':
    _main()
