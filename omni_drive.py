import sys
import time
import signal
import sched
import pickle

import matplotlib.pyplot as plt
import numpy as np
import RPi.GPIO as GPIO
from scipy.optimize import curve_fit
from typing import Callable, Any, Union, Tuple, List
import simple_pid
import sshkeyboard
from collections import namedtuple

from actuator import LinActuator
from motion import MotionSensor, MotionSensors
from player_movement import PlayerMovement, Feedback


class OmniDrive:

    # FB_FUN_TYPE =
    PIDp = namedtuple('PIDp', ['Kp', 'Ki', 'Kd'])
    AXES = 3  # forward/backward, left/right strafe, left/right turn
    PWM_2_MOTION_FUN = lambda x, *p: x * p[0] + p[1]  # uses pwm_to_motion_p params for each axis

    def __init__(self, roller_pins, lin_act_pins, up_trans_t=6, down_trans_t=5, pwm_freq=1000, calib_path=None):

        self.roller_dirs = np.array(['left', 'right'])
        self.simple_dirs_v = {'forward': [1., 0., 0.], 'backward': [-1., 0., 0.],  # v, vn, w
                              'left_strafe': [0., -1., 0.], 'right_strafe': [0., 1., 0.],
                              'left_turn': [0., 0., -1.], 'right_turn': [0., 0., 1.]}
        self.simple_dirs_v = {k: np.array(v) for k, v in self.simple_dirs_v.items()}

        self.noise_floors = np.zeros(OmniDrive.AXES)  # noise/second; 3 axes
        self.pwm_to_motion_p = np.stack([np.ones(OmniDrive.AXES), np.zeros(OmniDrive.AXES)], axis=1)  # lin fun
        self.pwm_to_motion_scaler = np.ones(OmniDrive.AXES)  # scaler to translate motion/sec; no bias like above here

        self.motion_per_rad = 0  # amount of motion detected for 1 rad turn
        self.motion_per_cm = 0  # same but in cms (measured only for turn, but should generalize)
        self.drive_2_game_vel = [None, None, None]  # for each axis
        self.drive_2_game_acc = [None, None, None]  # how drive speed translates to in-game acceleration

        # pid stuff
        self.pid_p = [OmniDrive.PIDp(0., 0., 0.) for _ in range(OmniDrive.AXES)]  # PID parameters
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
            self.pwm_to_motion_scaler = calib['pwm_to_motion_scaler']
            self.pid_p = calib['pid_p']
            self.pid_dt = calib['pid_dt']
            self.pid_err_scalers = calib['pid_err_scalers']
            self.trans_mx = calib['trans_mx']
            self.motion_per_rad = calib['motion_per_rad']
            self.motion_per_cm = calib['motion_per_cm']
            self.drive_2_game_vel = calib['drive_2_game_vel']
            self.drive_2_game_acc = calib['drive_2_game_acc']

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
                 'pid_p': self.pid_p, 'pid_dt': self.pid_dt, 'pid_err_scalers': self.pid_err_scalers,
                 'trans_mx': self.trans_mx, 'motion_per_rad': self.motion_per_rad, 'motion_per_cm': self.motion_per_cm,
                 'drive_2_game_vel': self.drive_2_game_vel, 'drive_2_game_acc': self.drive_2_game_acc,
                 'pwm_to_motion_scaler': self.pwm_to_motion_scaler}

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

    def simple_drive(self, simple_dir: str, speed: float = 1., t=None, blocking=False, callback=None):
        if simple_dir not in self.simple_dirs_v:
            print(f'Direction {simple_dir} is not any of: {", ".join(self.simple_dirs_v)}!', file=sys.stderr)
            return
        print(f'Drive {simple_dir} for {t} seconds')

        drive_v = self.simple_dirs_v[simple_dir] * speed
        wheel_dir, wheel_dc = self.calc_wheel_v(drive_v)
        return self.drive(wheel_dir, wheel_dc, t, blocking, callback)

    def roll(self, feedback: Feedback, eps: float, blocking=False, callback: Callable[[], Any] = None):
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
        pids = [simple_pid.PID(*self.pid_p[axis], sample_time=self.pid_dt, output_limits=(-1., 1.), setpoint=0.,
                               error_map=lambda x: x * self.pid_err_scalers[axis])
                for axis in range(OmniDrive.AXES)]

        if blocking:
            neg_err = -feedback()

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
            neg_err = -feedback()  # the negative error, which enables the constant 0 setpoint

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
        dirs_to_try = ['forward', 'backward', 'left_turn', 'right_turn']  # TODO try strafe l/r w/ other motion sensor (use MotionSensorS class)

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
                motion = [motion[0], 0, motion[1]]  # f/b and w, TODO 0 -> no strafe yet -> just rewrite this function w/ 3 DoF
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

    def calibrate_speed(self):  # calling this is not necessary, only sets pwm_to_motion_p
        # try different speeds, compare with motion sensor recording,
        # establish drive velocity (motor pwm) to motion velocity function for each axis
        drive_t = 5  # s
        pwms_to_try = np.linspace(0.1, 1.0, 10)
        dirs_to_try = ['forward', 'backward', 'left_turn', 'right_turn']  # TODO strafe -> just rewrite this function w/ 3 DoF
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
                motion = [motion[0], 0, motion[1]]  # TODO add strafe -> just rewrite this function w/ 3 DoF
                motions.append(motion)

        # establish pwm to motion velocity function for each axis
        #   do scipy function fitting for each axis
        drive_vs = np.abs(np.array(drive_vs))  # abs: direction agnostic within axis
        motions = np.abs(np.array(motions))  # same
        axes = np.argmax(np.abs(drive_vs), axis=1)

        for axis in [0, 2]:  # TODO add strafe and [0, 1, 2] -> just rewrite this function w/ 3 DoF
            dv = drive_vs[axes == axis, axis]  # pwms (see scaling above)
            mv = motions[axes == axis, axis] / drive_t  # motion/sec
            p0 = np.ones(2)

            popt, pcov = curve_fit(OmniDrive.PWM_2_MOTION_FUN, dv, mv, p0)
            self.pwm_to_motion_p[axis, :] = popt
            self.pwm_to_motion_scaler[axis] = np.mean(mv / dv)

            print(f'axis {axis}) popt: {popt.tolist()}')

    def calibrate_full_rot(self, ball_r):  # TODO test
        # defines motion to angle (rad) transfer function
        print('Press the \'right\' key until a full rotation occurs; press \'left\' to correct; press Esc to finish')

        drive_v = .5
        nruns = 5
        flo = MotionSensor(0, 'front')
        # flo = MotionSensors(MotionSensor(0, 'front'), MotionSensor(1, 'back'))  # TODO MotionSensorS

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
            print(f'{run + 1}/{nruns} round')
            flo.get_rel_motion()  # reset rel motion
            sshkeyboard.listen_keyboard(
                on_press=_press,
                on_release=_release,
                until='esc'
            )
            complete_turn.append(flo.get_rel_motion())

        complete_turn = np.mean(complete_turn, axis=0)[-1]  # could use rest of complete turn for noise floor estimation

        self.motion_per_rad = complete_turn / (2 * np.pi)
        self.motion_per_cm = complete_turn / (2 * ball_r * np.pi)

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

        # cm_per_game_mov_unit: length in cm of 1 in-game distance unit
        # turn in-game error into motion error, by first translating it into cm, then to motion
        self.pid_err_scalers[[0, 1]] = cm_per_game_dist_unit * self.motion_per_cm  # -> err * this => motion err
        self.pid_err_scalers[2] = self.motion_per_rad  # turn in rad -> turn in motion

        # use pid_dt and motion/sec at drive speed of 1 to init Kp,
        #    such that controller outputs a drive that results in a motion under pid_dt equaling (at most) err,
        #    so output = 1 when err_mot >= pid_dt * motion_per_sec (latter is pwm_to_motion_scaler)
        Kps = 1. / (self.pwm_to_motion_scaler * self.pid_dt)
        for axis in range(OmniDrive.AXES):
            # set very low Ki and Kd just to play around  # TODO test
            self.pid_p[axis] = OmniDrive.PIDp(Kps[axis], Kps[axis] / 20., Kps[axis] / 20.)

        # TODO implement actual incremental calibration as described above


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


def _onmni_test():
    calib_path = sys.argv[1] if len(sys.argv) > 1 else None

    lin_act_pins = {'up': 22, 'down': 4, 'enable': 27}
    roller0_pins = {'right': 23, 'left': 24, 'pwm': 18}
    roller1_pins = {'right': 5, 'left': 6, 'pwm': 13}
    roller2_pins = {'right': 25, 'left': 26, 'pwm': 12}
    roller_pins = [roller0_pins, roller1_pins, roller2_pins]

    omni_drive = OmniDrive(roller_pins, lin_act_pins, up_trans_t=4, down_trans_t=4, pwm_freq=1000, calib_path=calib_path)
    omni_drive.setup()

    def exit_code(*args):
        omni_drive.cleanup()
        exit(0)

    signal.signal(signal.SIGINT, exit_code)

    # omni_drive.calibrate_transfer_fun2()

    # tests
    wheel_tests(omni_drive)

    omni_drive.cleanup()


def man_drive(speed):

    lin_act_pins = {'up': 22, 'down': 4, 'enable': 27}
    roller0_pins = {'right': 23, 'left': 24, 'pwm': 18}
    roller1_pins = {'right': 5, 'left': 6, 'pwm': 13}
    roller2_pins = {'right': 25, 'left': 26, 'pwm': 12}
    roller_pins = [roller0_pins, roller1_pins, roller2_pins]

    omni_drive = OmniDrive(roller_pins, lin_act_pins, up_trans_t=4, down_trans_t=4, pwm_freq=1000)
    omni_drive.setup()

    def exit_code(*args):
        omni_drive.cleanup()
        exit(0)

    signal.signal(signal.SIGINT, exit_code)

    class _key_listener:
        key_mapping = {'up': 'forward', 'down': 'backward', 'right': 'right_strafe', 'left': 'left_strafe',
                       'w': 'forward', 's': 'backward', 'd': 'right_strafe', 'a': 'left_strafe',
                       'k': 'left_turn', 'l': 'right_turn'}

        def _press(self, key):
            if key in self.key_mapping:
                self.current_dir += omni_drive.simple_dirs_v[self.key_mapping[key]]
                wheel_dir, wheel_dc = omni_drive.calc_wheel_v(self.current_dir * speed)
                omni_drive.drive(wheel_dir, wheel_dc)

        def _release(self, key):
            if key in self.key_mapping:
                self.current_dir -= omni_drive.simple_dirs_v[self.key_mapping[key]]
                if np.abs(self.current_dir).sum() > 1e-4:
                    wheel_dir, wheel_dc = omni_drive.calc_wheel_v(self.current_dir * speed)
                    omni_drive.drive(wheel_dir, wheel_dc)
                else:
                    print('STOP')
                    omni_drive.stop()

        def __init__(self):
            self.current_dir = np.zeros(3)
            print('Press Esc to quit, or up, down, left, right, K, or L to roll the ball..')
            sshkeyboard.listen_keyboard(
                on_press=self._press,
                on_release=self._release,
            )

    _key_listener()
    omni_drive.cleanup()


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'man':
        man_drive(float(sys.argv[2]) if len(sys.argv) > 2 else .7)
    else:
        _onmni_test()
