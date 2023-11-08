import sys
import time
import signal
import sched
import pickle
from importlib.util import find_spec
from pprint import pprint

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
from typing import Callable, Any, Union, Tuple, List
import simple_pid
import sshkeyboard
import keyboard

if find_spec('RPi'):
    import RPi.GPIO as GPIO
else:
    GPIO = None

from actuator import LinActuator
from motion import MotionSensor, MotionSensors
from player_movement import PlayerMovement, Feedback
from config import *
from common import *


class OmniDrive:
    LIN_ACT_PINS = {'up': 22, 'down': 4, 'enable': 27}
    ROLLER0_PINS = {'right': 23, 'left': 24, 'pwm': 18}
    ROLLER1_PINS = {'right': 5, 'left': 6, 'pwm': 13}
    ROLLER2_PINS = {'right': 25, 'left': 26, 'pwm': 12}
    ROLLER_PINS = [ROLLER0_PINS, ROLLER1_PINS, ROLLER2_PINS]

    KEYBOARD_MAPPING = {'up': 'forward', 'down': 'backward', 'right': 'right_turn', 'left': 'left_turn',
                        'w': 'forward', 's': 'backward', 'd': 'right_strafe', 'a': 'left_strafe',
                        'k': 'left_turn', 'l': 'right_turn'}

    # FB_FUN_TYPE =
    AXES = 3  # forward/backward, left/right strafe, left/right turn
    PWM_2_MOTION_FUN = lambda x, *p: x * p[0] + p[1]  # uses pwm_to_motion_p params for each axis
    SIMPLE_DIRS_V = {k: np.array(v) for k, v in {'forward': [1., 0., 0.], 'backward': [-1., 0., 0.],  # v, vn, w
                                                 'left_strafe': [0., -1., 0.], 'right_strafe': [0., 1., 0.],
                                                 'left_turn': [0., 0., -1.], 'right_turn': [0., 0., 1.]}.items()}

    def __init__(self, roller_pins=ROLLER_PINS, lin_act_pins=LIN_ACT_PINS, up_trans_t=4, down_trans_t=4,
                 auto_mounting=False, mount_init=False, pwm_freq=100, calib_path=None):

        self.roller_dirs = np.array(['left', 'right'])
        # self.simple_dirs_v = {'forward': [1., 0., 0.], 'backward': [-1., 0., 0.],  # v, vn, w
        #                       'left_strafe': [0., -1., 0.], 'right_strafe': [0., 1., 0.],
        #                       'left_turn': [0., 0., -1.], 'right_turn': [0., 0., 1.]}
        # self.simple_dirs_v = {k: np.array(v) for k, v in self.simple_dirs_v.items()}

        self.noise_floors = np.zeros(OmniDrive.AXES)  # noise/second; 3 axes
        self.pwm_to_motion_p = np.stack([np.ones(OmniDrive.AXES), np.zeros(OmniDrive.AXES)], axis=1)  # lin fun
        self.pwm_to_motion_scaler = np.ones(OmniDrive.AXES)  # scaler to translate motion/sec; no bias like above here
        self.pwm_to_motion_min_pwm = np.zeros(OmniDrive.AXES)

        # last calibration:
        self.noise_floors = np.array([918.89, 2866.23, 1487.46])
        self.pwm_to_motion_min_pwm = np.array([0.4, 0.6, 0.3])
        self.pwm_to_motion_p = np.array([[10652.57, -3037.57], [13279.69, -5839.49], [8550.87, -545.12]])
        self.pwm_to_motion_scaler = np.array([5795.90, 5714.26, 7259.36])

        self.motion_per_rad = None  # amount of motion detected for 1 rad turn
        self.motion_per_cm = None  # same but in cms (measured only for turn, but should generalize)
        # motion_per_rad: 150.2422662787492
        # motion_per_cm: 7.51211331393746
        self.drive_2_game_vel = [None, None, None]  # for each axis
        self.drive_2_game_acc = [None, None, None]  # how drive speed translates to in-game acceleration

        # pid stuff
        self.pid_p = [PIDp(0., 0., 0.) for _ in range(OmniDrive.AXES)]  # PID parameters
        self.pid_dt = 0.05  # sec; time constant, time between PID updates
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
        self.auto_mounting = auto_mounting
        self.mount_init = mount_init
        self.mounted = None

        self.driving = False
        self.rolling = False
        self.roll_fun: Callable[[OmniDrive], Any] = lambda x: None
        self.current_drive_v: np.ndarray = np.zeros(OmniDrive.AXES)
        self.scheduler = sched.scheduler(time.time, time.sleep)

        self.lin_act = LinActuator(lin_act_pins)

        # update already calibrated vars
        if calib_path:
            with open(calib_path, 'rb') as f:
                calib = pickle.load(f)
            self.noise_floors = calib['noise_floors']
            self.pwm_to_motion_p = calib['pwm_to_motion_p']
            self.pwm_to_motion_scaler = calib['pwm_to_motion_scaler']
            self.pwm_to_motion_min_pwm = calib['pwm_to_motion_min_pwm']
            self.pid_p = calib['pid_p']
            self.pid_dt = calib['pid_dt']
            self.pid_err_scalers = calib['pid_err_scalers']
            self.trans_mx = calib['trans_mx']
            self.motion_per_rad = calib['motion_per_rad']
            self.motion_per_cm = calib['motion_per_cm']
            self.drive_2_game_vel = calib['drive_2_game_vel']
            self.drive_2_game_acc = calib['drive_2_game_acc']

        self.got_set_up = False
        GPIO.setmode(GPIO.BCM)

    # getters: nice for reaching variables over socket com
    # (shut up, properties are a hustle, not gonna rename my vars)

    # def get_simple_dir_v(self, dir):
    #     return self.simple_dirs_v[dir]

    def is_mounted(self):
        return self.mounted

    def setup(self):  # call before using OmniDrive
        if self.got_set_up:
            print('OmniDrive already got set up, running .setup again..', file=sys.stderr)

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

        if self.mount_init:  # start in unmounted = rat movement position
            self.mount(blocking=True)  # make sure the structure won't be raised out-of-bounds
            self.letgo(blocking=True)
        self.mounted = False

        self.got_set_up = True

    def save(self, calib_path):
        calib = {'noise_floors': self.noise_floors, 'pwm_to_motion_p': self.pwm_to_motion_p,
                 'pid_p': self.pid_p, 'pid_dt': self.pid_dt, 'pid_err_scalers': self.pid_err_scalers,
                 'trans_mx': self.trans_mx, 'motion_per_rad': self.motion_per_rad, 'motion_per_cm': self.motion_per_cm,
                 'drive_2_game_vel': self.drive_2_game_vel, 'drive_2_game_acc': self.drive_2_game_acc,
                 'pwm_to_motion_scaler': self.pwm_to_motion_scaler, 'pwm_to_motion_min_pwm': self.pwm_to_motion_min_pwm}

        with open(calib_path, 'wb') as f:
            pickle.dump(calib, f)

    def _set_mount_pos(self, mounted):  # used by mount() and letgo()
        self.mounted = mounted

    def mount(self, blocking=False, callback=None):
        if self.mounted:
            return
        self.lin_act.drive('down', self.down_trans_t, blocking=blocking)
        if blocking:
            self.mounted = True
        else:
            self.scheduler.enter(self.down_trans_t, 1, self._set_mount_pos, kwargs={'mounted': True})
        if callback is not None:
            self.scheduler.enter(self.down_trans_t, 2, callback)

    def letgo(self, blocking=False, callback=None):
        if not self.mounted:
            return
        self.lin_act.drive('up', self.up_trans_t, blocking=blocking)
        if blocking:
            self.mounted = False
        else:
            self.scheduler.enter(self.up_trans_t, 1, self._set_mount_pos, kwargs={'mounted': False})
        if callback is not None:
            self.scheduler.enter(self.up_trans_t, 2, callback)

    def calc_wheel_v(self, drive_v, ret_wheel_v=False):
        highest_abs_v = np.abs(drive_v).max()
        if highest_abs_v > 1.:
            print(f'Velocity values should be in the range [-1,1]: {drive_v.tolist()}!', file=sys.stderr)
            drive_v /= highest_abs_v

        # flip forward-backward
        drive_v = np.array(drive_v)  # cpy first
        drive_v[0] = -1 * drive_v[0]

        # translate wheel_v to ignore low pwms that would not move the ball: [0,1] -> [min_pwm,1]
        # leave 0s as 0s tho
        abs_drive_v = np.abs(drive_v)
        abs_drive_v = (self.pwm_to_motion_min_pwm + abs_drive_v * (1. - self.pwm_to_motion_min_pwm)) * (abs_drive_v > 0)
        drive_v = abs_drive_v * np.sign(drive_v)  # +-

        wheel_v = np.matmul(self.trans_mx, drive_v.transpose())
        wheel_v_normed = wheel_v / np.abs(wheel_v).max() * highest_abs_v  # normalize then scale back to requested velocity

        wheel_dc = np.abs(wheel_v_normed * 100.)  # duty cycle max 100
        wheel_dir = self.roller_dirs[(wheel_v_normed > 0).astype(int)]

        if ret_wheel_v:
            return wheel_dir, wheel_dc, wheel_v, wheel_v_normed
        return wheel_dir, wheel_dc

    def drive(self, wheel_dir, wheel_dc, t=None, blocking=False, unmount=False, callback=None):
        if self.auto_mounting and not self.mounted:
            self.mount()

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
                self.stop(unmount)
        elif t is not None and not blocking:
            self.scheduler.enter(t, 1, self.stop, kwargs={'unmount': unmount})

        if t is not None and callback is not None:
            self.scheduler.enter(t, 2, callback)

        return True

    def simple_drive(self, simple_dir: str, speed: float = 1., t=None, blocking=False, unmount=False, callback=None):
        if simple_dir not in self.SIMPLE_DIRS_V:
            print(f'Direction {simple_dir} is not any of: {", ".join(self.SIMPLE_DIRS_V)}!', file=sys.stderr)
            return
        print(f'Drive {simple_dir} for {t} seconds at speed {speed:.2f}')

        drive_v = self.SIMPLE_DIRS_V[simple_dir] * speed
        wheel_dir, wheel_dc = self.calc_wheel_v(drive_v)
        self.current_drive_v = drive_v
        return self.drive(wheel_dir, wheel_dc, t, blocking, unmount, callback)

    def roll(self, feedback: Feedback, eps: float, blocking=False, unmount=False, callback: Callable[[], Any] = None):
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

        # feedback returns the error
        # no need to provide the setpoint, set it to 0, and take err from feedback() and negate it

        if self.driving or self.rolling:
            print(f'OmniDrive already running (driving={self.driving}, rolling={self.rolling})! '
                  f'Call stop first!', file=sys.stderr)
            return False

        # set up PIDs
        # no need for proportional_on_measurement
        # TODO the err scaler converts game error to motion error
        pids = [simple_pid.PID(*self.pid_p[axis], sample_time=self.pid_dt, output_limits=(-1., 1.), setpoint=0.,
                               error_map=lambda x: x * self.pid_err_scalers[axis])
                for axis in range(OmniDrive.AXES)]

        if blocking:
            neg_err = -feedback()

            while np.abs(np.max(neg_err)) < eps or not self.rolling:
                drive_v = np.array([pid(neg_err[axes]) for axes, pid in enumerate(pids)])
                wheel_dir, wheel_dc = self.calc_wheel_v(drive_v)
                self.drive(wheel_dir, wheel_dc)
                self.current_drive_v = drive_v

            self.stop(unmount)
            if callback:
                callback()

            return True

        # not blocking
        # set up function to run in loop that returns drive_v
        def _roll_fun(od: OmniDrive):
            neg_err = -feedback()  # the negative error, which enables the constant 0 setpoint

            # when done
            if np.abs(np.max(neg_err)) < eps or not od.rolling:
                od.stop(unmount)  # od.rolling -> False
                if callback:
                    callback()
                return

            # roll on
            drive_v = np.array([pid(neg_err[axes]) for axes, pid in enumerate(pids)])
            wheel_dir, wheel_dc = od.calc_wheel_v(drive_v)
            od.drive(wheel_dir, wheel_dc)
            od.current_drive_v = drive_v

        self.roll_fun = _roll_fun
        self.rolling = True
        return True

    def stop(self, unmount=False):
        for i, pins in enumerate(self.roller_pins):
            GPIO.output(pins['left'], GPIO.LOW)
            GPIO.output(pins['right'], GPIO.LOW)
            self.pwms[i].stop()

        self.driving = False
        self.rolling = False
        self.current_drive_v = np.zeros(OmniDrive.AXES)

        if unmount and self.auto_mounting and self.mounted:
            self.letgo()

    def loop(self):
        self.scheduler.run(blocking=False)
        self.lin_act.loop()

        if self.rolling:
            self.roll_fun(self)

    def cleanup(self):
        self.stop(unmount=self.auto_mounting)
        self.lin_act.cleanup(gpio_cleanup=False)
        GPIO.cleanup()
        print('OmniDrive cleanup done')

    def motion_to_phys(self, movement: np.ndarray):
        # scales motion sensor detected movement into physical units (cm and rad)
        # only works properly if calibrate_full_rot() has run i.e. motion_per_rad motion_per_cm are defined
        movement = np.array(movement)
        movement[:-1] /= self.motion_per_cm
        movement[-1] /= self.motion_per_rad
        return movement

    def calibrate_transfer_fun(self):
        import torch
        import omni_transfer_opt

        flo1 = MotionSensor(**FRONT_MOTION_PARAMS)
        flo2 = MotionSensor(**SIDE_MOTION_PARAMS)
        flo = MotionSensors(flo1, flo2)
        drive_t = 5
        niter = 50
        speed = 0.8
        dirs_to_try = ['forward', 'backward', 'left_strafe', 'right_strafe', 'left_turn', 'right_turn']

        # setup nn
        trans = omni_transfer_opt.OmniTransferOpt(self.def_d)
        loss = torch.nn.MSELoss()
        opt = torch.optim.SGD(trans.parameters(), lr=0.01)
        errs = []
        trans_mxs = []

        print('Transfer function calibration begins..')
        for it in range(niter):
            print(f'{it}.', '-' * 10)

            # record actual motion
            actual_motion = []
            wheel_vs = []

            for dir_i, dir_ in enumerate(dirs_to_try):

                drive_v = self.SIMPLE_DIRS_V[dir_] * speed
                print(f'go {dir_} at {drive_v}')
                wheel_dir, wheel_dc, wheel_v, wheel_v_normed = self.calc_wheel_v(drive_v, ret_wheel_v=True)
                wheel_vs.append(wheel_v)

                flo.loop()
                flo.get_rel_motion()  # zero out rel vars
                self.drive(wheel_dir, wheel_dc, t=drive_t, blocking=True)

                flo.loop()
                motion = flo.get_rel_motion()
                motion = motion / np.abs(motion).sum()
                actual_motion.append(motion)

            # evaluate model for expected motion
            actual_motion = torch.reshape(torch.tensor(actual_motion, dtype=torch.float32), (len(dirs_to_try), OmniDrive.AXES, 1))
            wheel_vs = torch.reshape(torch.tensor(np.array(wheel_vs), dtype=torch.float32), (len(dirs_to_try), 3, 1))
            expected_motion = trans(wheel_vs)
            expected_motion[:, 0, :] *= -1  # front-back, and left-right strafe are flipped

            print('WHEEL_VS')
            print(torch.round(wheel_vs[..., 0], decimals=2))
            print('MOTION')
            print(torch.round(actual_motion[..., 0], decimals=2))
            print('EXPECTED MOTION')
            print(torch.round(expected_motion[..., 0], decimals=2))

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

    def calibrate_speed(self):  # sets pwm_to_motion_p and pwm_to_motion_scaler
        # try different speeds, compare with motion sensor recording,
        # establish drive velocity (motor pwm) to motion velocity function for each axis
        nrep = 2  # how many repetitions in each axis/dir
        drive_t = 5  # s
        pwms_to_try = np.linspace(0.1, 1.0, 10)
        dirs_to_try = ['forward', 'backward', 'left_strafe', 'right_strafe', 'left_turn', 'right_turn']

        flo1 = MotionSensor(**FRONT_MOTION_PARAMS)
        flo2 = MotionSensor(**SIDE_MOTION_PARAMS)
        flo = MotionSensors(flo1, flo2)

        drive_vs = []
        motions = []

        for dir_i, dir_ in enumerate(dirs_to_try):
            print(f'{nrep} x direction:', dir_)
            for _ in range(nrep):
                for pwm in pwms_to_try:
                    print(f'pwm: {pwm:.2f} ...', end=' ', flush=True)

                    drive_v = self.SIMPLE_DIRS_V[dir_] * pwm
                    wheel_dir, wheel_dc = self.calc_wheel_v(drive_v)
                    drive_vs.append(drive_v)

                    flo.loop()
                    flo.get_rel_motion()  # zero out rel vars
                    self.drive(wheel_dir, wheel_dc, t=drive_t, blocking=True)

                    flo.loop()
                    motion = flo.get_rel_motion()
                    motions.append(motion)
                    print(f'motion: {motion}', end='\n')

        # establish pwm to motion velocity function for each axis
        #   do scipy function fitting for each axis
        drive_vs = np.abs(np.array(drive_vs))  # abs: direction agnostic within axis
        motions = np.abs(np.array(motions))  # same
        dom_axes = np.argmax(np.abs(drive_vs), axis=1)  # dominant axis of driving

        # experimentally: anything below 500 motion is noise; the function of drive to motion looks like a ReLU;
        #   find highest drive_vs where the motion is close to 0, i.e. <500, call it low_end
        #   from low_end to max(drive_vs) fit a linear function;
        #   don't drive wheels below a drive of low_end --> scale drive_vs of [0,1] to [low_end, 1] later
        print('pwm to motion:')
        for axis in range(OmniDrive.AXES):

            dv = drive_vs[dom_axes == axis, axis]  # pwms (see scaling above)
            mv = motions[dom_axes == axis, axis] / drive_t  # motion/sec
            noise_motion = motions[dom_axes != axis, axis].mean()  # noise floor for motion
            print(f'AXIS {axis}:\n\tdv: {dv.tolist()}\n\tmv: {mv.tolist()}\n\tnoise: {noise_motion:.02f}')

            # find low_end for each direction in an axis
            dv1, dv2 = dv[:len(dv) // 2], dv[len(dv) // 2:]
            mv1, mv2 = mv[:len(mv) // 2], mv[len(mv) // 2:]

            low_end1_i = np.max(np.arange(len(mv1))[mv1 < noise_motion])
            low_end2_i = np.max(np.arange(len(mv2))[mv2 < noise_motion])

            # the actual low_end is the avg of the two directions, to have one dv to mv function for each axis
            low_end = (dv1[low_end1_i] + dv2[low_end2_i]) / 2
            self.pwm_to_motion_min_pwm[axis] = low_end

            # do one curve fit for all >low_end
            p0 = np.ones(2)  # slope and bias
            dvf = np.concatenate([dv1[low_end1_i:], dv2[low_end2_i:]])
            mvf = np.concatenate([mv1[low_end1_i:], mv2[low_end2_i:]])
            popt, pcov = curve_fit(OmniDrive.PWM_2_MOTION_FUN, dvf, mvf, p0)

            self.pwm_to_motion_p[axis, :] = popt
            self.pwm_to_motion_scaler[axis] = np.mean(mvf / dvf)
            self.noise_floors[axis] = noise_motion

        print('pwm_to_motion_min_pwm:', self.pwm_to_motion_min_pwm)
        print('pwm_to_motion_p:', self.pwm_to_motion_p)
        print('pwm_to_motion_scaler:', self.pwm_to_motion_scaler)

    def calibrate_full_rot(self, ball_r):  # ball_r in cm
        # defines motion to angle (rad) transfer function
        print('Press the \'right\' key until a full rotation occurs; press \'left\' to correct; press Esc to finish')

        drive_v = 0.75
        nruns = 5

        flo1 = MotionSensor(**FRONT_MOTION_PARAMS)
        flo2 = MotionSensor(**SIDE_MOTION_PARAMS)
        flo = MotionSensors(flo1, flo2)

        def _press(key):
            if key == 'right':
                self.simple_drive('right_turn', speed=drive_v)
            elif key == 'left':
                self.simple_drive('left_turn', speed=drive_v)
            flo.loop()

        def _release(key):
            if key in ('left', 'right'):
                self.stop()
            flo.loop()

        complete_turn = []
        for run in range(nruns):
            print(f'{run + 1}/{nruns} round: press right up to a full 360-degree turn')
            flo.get_rel_motion()  # reset rel motion
            sshkeyboard.listen_keyboard(
                on_press=_press,
                on_release=_release,
                until='esc'
            )
            complete_turn.append(flo.get_rel_motion()[2])  # only get the turn axis

        complete_turn = np.mean(complete_turn)

        self.motion_per_rad = complete_turn / (2 * np.pi)
        self.motion_per_cm = complete_turn / (2 * ball_r * np.pi)

        print('motion_per_rad:', self.motion_per_rad)
        print('motion_per_cm:', self.motion_per_cm)

    # TODO test run
    def calibrate_game_movement(self, feedback: Feedback, set_rel_goal: Callable[[np.ndarray], Any],
                                map_size: Tuple[float, float]):

        fb = np.array([[1., 0., 0.], [-1., 0., 0.]])  # forward/backward
        fb_drive_2_game_vel, fb_drive_2_game_acc = self._calibrate_game_movement(fb, feedback, set_rel_goal, map_size)

        slr = np.array([[0., 1., 0.], [0., -1., 0.]])  # strafe left/right
        slr_drive_2_game_vel, slr_drive_2_game_acc = self._calibrate_game_movement(slr, feedback, set_rel_goal, map_size)

        self.drive_2_game_vel = [fb_drive_2_game_vel, slr_drive_2_game_vel, None]  # 3rd axis (turn) not applicable
        self.drive_2_game_acc = [fb_drive_2_game_acc, slr_drive_2_game_acc, None]

    def _calibrate_game_movement(self, dirs: np.ndarray, feedback: Feedback,
                                 set_rel_goal: Callable[[np.ndarray], Any], map_size: Tuple[float, float]):
        # feedback() returns the error from the set goal
        # set_rel_goal() sets a relative goal

        # Doom player has its sliding acceleration and deceleration (sliding ice-cube) with forces_applied - friction
        #   and forces_applied is defined by the recorded ball motion
        # acceleration and deceleration needs to be incorporated in rolling:
        #   calculate acceleration, deceleration (same as accel?) and the stable velocity from drive_v
        # put the player in an arena then call this function
        # A) omni driver runs drive() in straight directions xor turns and tracks in-game position (and vel., accel.)
        #    calc drive velocity to game velocity function
        # B) derive acceleration, deceleration and stable in-game velocity from drive velocity (if dependent on it)

        # FOR NOW B) DECELERATION IS IGNORED FOR NOW AS SLIDING IN DOOM CAN BE MINIMIZED WITH THIS SCRIPT:
        #   https://forum.zdoom.org/viewtopic.php?p=933227&sid=fdef5121aec04509a2b76c8048ea5cc5#p933227

        # A) and B) derive drive_v to game_v transfer function (scaler), and acceleration
        # go straight, go backwards, go straight, ... until enough runs
        nruns = 12
        drive_v_speed_rng = (0.1, 1.0)
        drive_vs, vels, accs = [], [], []

        assert len(np.unique(np.argmax(dirs, axis=1))) == 1, 'move only on 1 axis'
        axis = np.argmax(dirs[0])

        for r in range(nruns):
            dir_ = dirs[r % len(dirs)]
            rel_goal = dir_ * map_size[axis] / 2
            drive_v_speed = np.random.uniform(*drive_v_speed_rng)
            dvs_per_run, vels_per_run, accs_per_run = [], [], []

            set_rel_goal(rel_goal)
            fb = feedback()
            self.drive(*self.calc_wheel_v(dir_ * drive_v_speed))
            # self.simple_drive('forward' if rel_goal[0] > 0 else 'backward', drive_v_speed)

            while (rel_goal[0] > 0 and fb[0] > 0) or (rel_goal[0] < 0 and fb[0] < 0):
                dvs_per_run.append(drive_v_speed)
                vels_per_run.append(feedback.vel().abs()[axis])
                accs_per_run.append(feedback.acc().abs()[axis])
                time.sleep(self.pid_dt)  # use same dt, as the pid controller sample time
                fb = feedback()

            self.stop()
            drive_vs.append(np.array(dvs_per_run))
            vels.append(np.array(vels_per_run))
            accs.append(np.array(accs_per_run))

            time.sleep(1)
            rel_goal *= -1

        # filter velocities to ignore acceleration times; select max acceleration as the correspondent value to drive_v
        # assumed: movement starts w/ acceleration (deceleration is not recorded), then velocity is ~constant, acc ~= 0;
        #          most of the movement is constant velocity
        filt_dvs_v, filt_dvs_a, filt_vels, filt_accs = [], [], [], []  # separate drive_v for vel and acc
        for r in range(nruns):
            dv, vel, acc = drive_vs[r], vels[r], accs[r]

            max_acc_i = np.argmax(acc)  # already abs
            acc_i = np.arange(len(acc))
            zero_crossings_after_max = acc_i[(acc_i > max_acc_i) & (acc <= 0)]
            first_zero_crossing_after_max = zero_crossings_after_max[np.argmin(np.abs(zero_crossings_after_max - max_acc_i))]

            filt_dvs_v.append(dv[first_zero_crossing_after_max:])
            filt_vels.append(vel[first_zero_crossing_after_max:])
            filt_dvs_a.append(dv[max_acc_i])  # let's hope the noise avgs out
            filt_accs.append(acc[max_acc_i])

        filt_dvs_v = np.concatenate(filt_dvs_v)
        filt_vels = np.concatenate(filt_vels)
        filt_dvs_a = np.array(filt_dvs_a)
        filt_accs = np.array(filt_accs)

        # fit
        drive_2_game_vel = (filt_vels / filt_dvs_v).mean()
        drive_2_game_acc = (filt_accs / filt_dvs_a).mean()

        # show fit
        plt.plot(filt_dvs_v, filt_vels, label='drive to vel')
        plt.plot(filt_dvs_a, filt_accs, label='drive to acc')

        x = np.linspace(filt_dvs_v.min(), filt_dvs_v.max(), 50)
        plt.plot(x, x * drive_2_game_vel, '--', label='drive to vel fit')
        plt.plot(x, x * drive_2_game_acc, '--', label='drive to acc fit')

        plt.legend()
        plt.show()

        return drive_2_game_vel, drive_2_game_acc

    def calibrate_pid_params(self, cm_per_game_dist_unit: float):  # TODO test
        # optimize PID params
        #    the omni driver sets goals for itself (in game delta coordinates = fb error)
        #    omni driver runs roll() (either straight x,y motion or turns) to get to the goal,
        #      while tracking in-game motion
        #    optimizes Kp, Ki and Kd for each axis; init them according to translation (vel., accel.),
        #    change Kp, Ki, Kd up/down until reaching the fast conversion without overshooting

        # PID tuning: https://www.youtube.com/watch?v=sFOEsA0Irjs, https://www.youtube.com/watch?v=IB1Ir4oCP5k
        #   https://medium.com/@svm161265/when-and-why-to-use-p-pi-pd-and-pid-controller-73729a708bb5
        # calibration process:
        #   increase Kp to reduce rise time; keep increasing it until overshoot happens
        #   increase Ki to reduce rise time too (causes overshoot); helps in noisy environments -> set it to a low value
        #   increase Kd to decrease overshoot; can't be used when noise is present -> set it to a low value, maybe == Ki

        # cm_per_game_dist_unit: length in cm of 1 in-game distance unit
        # turn in-game error into motion error, by first translating it into cm, then to motion
        self.pid_err_scalers[[0, 1]] = cm_per_game_dist_unit * self.motion_per_cm  # -> err * this => motion err
        self.pid_err_scalers[2] = self.motion_per_rad  # turn in rad -> turn in motion

        # use pid_dt and motion/sec at drive speed of 1 to init Kp,
        #    such that controller outputs a drive that results in a motion under pid_dt equaling (at most) err,
        #    so output = 1 when err_mot >= pid_dt * motion_per_sec (latter is pwm_to_motion_scaler)
        Kps = 1. / (self.pwm_to_motion_scaler * self.pid_dt)
        for axis in range(OmniDrive.AXES):
            # set very low Ki and Kd just to play around  # TODO test
            self.pid_p[axis] = PIDp(Kps[axis], Kps[axis] / 10., Kps[axis] / 10.)

        # TODO implement actual incremental calibration as described above


def calibrate(omni_drive, calibration_path, **calib_kwargs):
    omni_drive.setup()
    omni_drive.mount()

    # calibrate
    print('Calibrate transfer function..')
    omni_drive.calibrate_transfer_fun()
    print('Calibrate speed..')
    omni_drive.calibrate_speed()
    print('Calibrate full rotation..')
    omni_drive.calibrate_full_rot(ball_r=20)
    # omni_drive.calibrate_game_movement()  # TODO implement !!!!
    print('Calibrate PID parameters..')
    omni_drive.calibrate_pid_params(calib_kwargs['cm_per_game_dist_unit'])

    # save results
    omni_drive.save(calibration_path)
    print('Calibration saved under:', calibration_path)


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


def onmni_test(omni_drive):
    wheel_tests(omni_drive)
    # TODO ...
    omni_drive.cleanup()


def ssh_man_drive(omni_drive, speed, check_motion=False):

    if check_motion:
        flo1 = MotionSensor(**FRONT_MOTION_PARAMS)
        flo2 = MotionSensor(**SIDE_MOTION_PARAMS)
        flo = MotionSensors(flo1, flo2)

    class _man_drive_key_listener:
        key_mapping = OmniDrive.KEYBOARD_MAPPING

        def _press(self, key):
            if key in self.key_mapping:
                if check_motion:
                    flo.loop()
                    flo.get_rel_motion()
                print(f'DRIVE {self.key_mapping[key]}..', end='', flush=True)
                self.current_dir += omni_drive.SIMPLE_DIRS_V[self.key_mapping[key]]
                wheel_dir, wheel_dc = omni_drive.calc_wheel_v(self.current_dir * speed)
                omni_drive.drive(wheel_dir, wheel_dc)

        def _release(self, key):
            if key in self.key_mapping:
                self.current_dir -= omni_drive.SIMPLE_DIRS_V[self.key_mapping[key]]
                if np.abs(self.current_dir).sum() > 1e-4:
                    wheel_dir, wheel_dc = omni_drive.calc_wheel_v(self.current_dir * speed)
                    omni_drive.drive(wheel_dir, wheel_dc)
                else:
                    omni_drive.stop()
                    if check_motion:
                        flo.loop()
                        rel_mot = flo.get_rel_motion()
                    else:
                        rel_mot = []
                    print(f'STOP after motion of {[int(m) for m in rel_mot]}', end='\n')

        def __init__(self):
            self.current_dir = np.zeros(OmniDrive.AXES)
            print('Press Esc to quit, or up, down, left, right, K, or L to roll the ball..')
            sshkeyboard.listen_keyboard(
                on_press=self._press,
                on_release=self._release,
            )

    _man_drive_key_listener()
    omni_drive.cleanup()


def local_man_drive(omni_drive: OmniDrive, init_speed=.7):
    key_mapping = OmniDrive.KEYBOARD_MAPPING

    print('drive keymapping:', flush=True)
    print('\n'.join([f'- {k}: {d}' for k, d in key_mapping.items()]))
    print('to decrease/increase speed press the 1 or 2 number keys')
    print('press space to mount/unmount omni drive')

    class _socket_drive():
        def __init__(self, omni_drive: OmniDrive, speed: float):
            self.omni_drive = omni_drive
            self.speed = speed
            self.prev_dir = np.zeros(OmniDrive.AXES)

        def __call__(self):  # call in a loop
            current_dir = np.zeros(OmniDrive.AXES)
            mount_state = 'stay'

            # compute direction
            for key, direction in key_mapping.items():
                if keyboard.is_pressed(key):
                    current_dir += OmniDrive.SIMPLE_DIRS_V[direction]

            # set volume
            if keyboard.is_pressed('1'):
                self.speed = max(.1, self.speed - .1)
                print('speed decreased to', self.speed)
            elif keyboard.is_pressed('2'):
                self.speed = min(1., self.speed + .1)
                print('speed increased to', self.speed)

            # (un)mount
            if keyboard.is_pressed('space'):
                if self.omni_drive.is_mounted():
                    self.omni_drive.letgo()
                    mount_state = 'letgo'
                else:  # was not mounted
                    self.omni_drive.mount()
                    mount_state = 'mounted'

            max_move = np.abs(current_dir).max()
            current_dir /= max_move if max_move > 0 else 1  # normalize to [-1, 1]

            if np.any(current_dir != self.prev_dir):
                self.prev_dir = current_dir
                if current_dir.sum() == 0:
                    self.omni_drive.stop()
                else:
                    wheel_dir, wheel_dc = self.omni_drive.calc_wheel_v(current_dir * self.speed)
                    self.omni_drive.drive(wheel_dir, wheel_dc)

            return current_dir, mount_state

    return _socket_drive(omni_drive, init_speed)


def main():
    # man args: speed calibration_path
    # calibrate args: calibration_path cm_per_game_dist_unit  # TODO argparse
    # test args: calibration_path
    function = sys.argv[1]  # must be 'man', 'calibrate' or 'test'
    assert function in ['man', 'man2', 'calib', 'test']

    def get_exit_code(omni_drive: OmniDrive):
        def _exit():
            omni_drive.cleanup()
            exit(0)
        return _exit

    if function.startswith('man'):
        speed = float(sys.argv[2]) if len(sys.argv) > 2 else .7
        calibration_path = sys.argv[3] if len(sys.argv) > 3 else None
        omni_drive = OmniDrive(up_trans_t=4, down_trans_t=4, calib_path=calibration_path, mount_init=False)
        omni_drive.setup()
        signal.signal(signal.SIGINT, get_exit_code(omni_drive))

        if function == 'man':
            ssh_man_drive(omni_drive, speed)
        elif function == 'man2':
            drive = local_man_drive(omni_drive, speed)
            while True:
                drive()

    elif function == 'calib':
        calibration_path = sys.argv[2]  # omni_calib.pckl
        cm_per_game_dist_unit = float(sys.argv[3])  # 3.81 for DOOM
        omni_drive = OmniDrive(up_trans_t=4, down_trans_t=4)
        omni_drive.setup()
        signal.signal(signal.SIGINT, get_exit_code(omni_drive))

        calibrate(omni_drive, calibration_path, cm_per_game_dist_unit=cm_per_game_dist_unit)

    elif function == 'test':
        calibration_path = sys.argv[2] if len(sys.argv) > 2 else None
        omni_drive = OmniDrive(up_trans_t=4, down_trans_t=4, calib_path=calibration_path)
        omni_drive.setup()
        signal.signal(signal.SIGINT, get_exit_code(omni_drive))

        onmni_test(omni_drive)


if __name__ == '__main__':
    # exit()
    main()
