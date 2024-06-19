import socket
import time
import traceback

import matplotlib.pyplot as plt
import numpy as np
from collections import deque

import vizdoom
from DOOM import DOOM
from motion import MotionSensor, MotionSensors, SmoothMotion
from actuator import LinActuator
from omni_drive import OmniDrive
from trainer import ManualTrainer
from player_movement import PlayerMovement, Feedback
from config import *
from live_testing import LiveLinePlot

from pi_wrapper import PiSmoothMotion, PiMotionSensor, PiMotionSensors, PiOmniDrive, PiFeedback, \
    PiPlayerMovement, ServerSocket, PiRewardCircuit


# pc/server address
host, port = '192.168.1.74', 4444  # TODO as cmd argument or feel automatically
reward_serial_port = '/dev/ttyACM0'  # ttyACM0

with ServerSocket(host, port) as conn:

    # setup VR
    flo1 = PiMotionSensor(conn, **FRONT_MOTION_PARAMS)
    flo2 = PiMotionSensor(conn, **SIDE_MOTION_PARAMS)
    flo = PiMotionSensors(conn, flo1, flo2)
    smooth_flo = PiSmoothMotion(conn, flo, 0.05, 100)

    pm = PlayerMovement()

    calibration_path = 'omni_calib.pckl'
    od = PiOmniDrive(conn, auto_mounting=False, mount_init=False, calib_path=calibration_path)
    od.setup()
    assert od.get('motion_per_cm') is not None and od.get('motion_per_rad') is not None

    reward_circuit = PiRewardCircuit(conn, reward_serial_port, auto_mixing_at_every=5,
                                     init_pressure_setpoint=1,  # TODO
                                     run_on_sep_thread=True)

    # setup game
    is_async = True
    player_mode = vizdoom.Mode.ASYNC_PLAYER if is_async else vizdoom.Mode.PLAYER
    cfg_update = dict(is_async=is_async, fullscreen=True, win_visible=True, res=vizdoom.ScreenResolution.RES_1024X576,
                      mode=player_mode, render_msgs=False, fov=140, post_set_res=None, repos_win=(1920, 0))
    doom = DOOM('doom/scenarios/arena_lowered.wad', 'map01', cfg_update)
    doom.game.set_ticrate(30)
    game_over = False

    # setup trainer
    trainer = ManualTrainer(doom, od, reward_circuit, move_r_per_sec=1, kill_r=30, man_r=10, # TODO
                            r_in_every=.8, min_r_given=10, omni_speed=.75, no_reward=False)

    # mov_live_plot = LiveLinePlot(nplots=3, ylim=(-1200, 1200))
    loop_ts, loop_tss = deque([], 100), deque([], 100)

    while not game_over:
        _start = time.time()

        # run VR devices
        t = time.time()
        od.loop()  # <2ms
        smooth_flo.loop()  # <3ms
        rc_state = reward_circuit.loop(verbose=False)  # >6ms
        t1 = time.time() - t

        # action
        t = time.time()
        mov = smooth_flo.get_vel()
        t2 = time.time() - t
        # mov_live_plot.update(time.time(), mov)

        lever = int(0 < rc_state['LEV'] < 100) if rc_state is not None else 0
        phys_mov = od.motion_to_phys(mov)
        action = (phys_mov, lever)

        # step
        t = time.time()
        state, reward, terminated, truncated, info = doom.step(action)
        game_over = terminated or truncated
        pm.loop(pos=np.array([state.position_x, state.position_y, state.angle]),
                vel=np.array([state.velocity_x, state.velocity_y]))
        t3 = time.time() - t

        # train
        t = time.time()
        trainer.enforce_action(info['step_i'], state)
        reward += trainer.give_reward(info['step_i'], state)
        t4 = time.time() - t

        # dispense rewards
        t = time.time()
        reward = max(0., reward)  # positive reinforcement only
        left_blow, right_blow = PiRewardCircuit.calc_puff_from_wall_bump(info['bump_angle'], info['bump_dist'])
        if reward > 0:
            print('REWARD:', reward)
        reward_circuit.update(valve_open_ms=reward, left_blow=left_blow > 0, right_blow=right_blow > 0)
        t5 = time.time() - t

        # benchmarking
        _end = time.time()
        if info['step_i'] > 50:
            loop_ts.append(_end - _start)
            loop_tss.append((t1, t2, t3, t4, t5))

        if info['step_i'] % 100 == 0:
            print('avg loop time:', np.mean(loop_ts) * 1000)
            print('tss:', [np.mean(ti) * 1000 for ti in zip(*loop_tss)])
            # plt.hist(loop_ts, bins=30)
            # plt.show()

    trainer.cleanup()
