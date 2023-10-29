import socket
import time
import traceback

import matplotlib.pyplot as plt
import numpy as np

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
host, port = '192.168.0.129', 4444  # TODO as cmd argument or feel automatically
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

    reward_circuit = PiRewardCircuit(conn, reward_serial_port, auto_mixing_at_every=5, run_on_sep_thread=True)  # TODO run_on_sep_thread=True test

    # setup game
    player_mode = vizdoom.Mode.PLAYER  # vizdoom.Mode.SPECTATOR | vizdoom.Mode.PLAYER
    cfg_update = dict(fullscreen=True, win_visible=True, res=vizdoom.ScreenResolution.RES_1024X576,
                      mode=player_mode, render_msgs=False, fov=140, post_set_res=None)  # TODO 16:8 '1024 512' ?
    doom = DOOM('doom/scenarios/arena_lowered.wad', 'map01', cfg_update)
    game_over = False

    # setup trainer
    trainer = ManualTrainer(doom, od, reward_circuit, move_r_per_sec=20, kill_r=100,
                            r_in_every=.3, min_r_given=10, omni_speed=.75)

    # mov_live_plot = LiveLinePlot(nplots=3, ylim=(-1200, 1200))
    loop_ts = []

    while not game_over:
        _start = time.time()

        # run VR devices
        od.loop()
        smooth_flo.loop()
        rc_state = reward_circuit.loop(verbose=False)  # TODO verbose False
        # if rc_state and rc_state['LEV'] > 0:
        #     print('lev:', rc_state['LEV'])

        # action
        mov = smooth_flo.get_vel()
        # mov_live_plot.update(time.time(), mov)

        lever = int(0 < rc_state['LEV'] < 300) if rc_state is not None else 0
        phys_mov = od.motion_to_phys(mov)
        action = (phys_mov, lever)

        # step
        state, reward, terminated, truncated, info = doom.step(action)
        game_over = terminated or truncated
        pm.loop(pos=np.array([state.position_x, state.position_y, state.angle]),
                vel=np.array([state.velocity_x, state.velocity_y]))

        # train
        if reward > 0:
            print('gameREWARD:', reward)
        trainer.enforce_action(info['step_i'], state)
        reward += trainer.give_reward(info['step_i'], state)

        # dispense rewards
        puff_cmd = PiRewardCircuit.calc_puff_from_wall_bump(info['bump_angle'], info['bump_dist'], return_cmd=True)
        reward = max(0., reward)  # positive reinforcement only
        if reward > 0:
            print('REWARD:', reward)
        reward_circuit.update(valve_open_ms=reward, **puff_cmd)  # TODO ms? ul? which one

        # benchmarking
        _end = time.time()
        if info['step_i'] > 50:
            loop_ts.append(_end - _start)

        if info['step_i'] % 100 == 0:
            print('avg loop time:', np.mean(loop_ts) * 1000)

    trainer.cleanup()
