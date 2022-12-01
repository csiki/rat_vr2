import socket
import time
import traceback
import numpy as np

from DOOM import DOOM
from motion import MotionSensor, MotionSensors, SmoothMotion
from actuator import LinActuator
from omni_drive import OmniDrive
from player_movement import PlayerMovement, Feedback
from config import *

from pi_wrapper import PiSmoothMotion, PiMotionSensor, PiMotionSensors, PiOmniDrive, PiFeedback, \
    PiPlayerMovement, ServerSocket, PiRewardCircuit


# pc/server address
host, port = '192.168.0.129', 4444  # '127.0.0.1'


# TODO check w sanyi not enough V going to roller motors ???


with ServerSocket(host, port) as conn:

    # setup VR
    flo1 = PiMotionSensor(conn, **FRONT_MOTION_PARAMS)
    flo2 = PiMotionSensor(conn, **SIDE_MOTION_PARAMS)
    flo = PiMotionSensors(conn, flo1, flo2)
    smooth_flo = PiSmoothMotion(conn, flo, 0.05)

    pm = PlayerMovement(do_calc_acc=True)

    calibration_path = 'omni_calib.pckl'
    od = PiOmniDrive(conn, mount_tracking=False, calib_path=calibration_path)  # TODO mount_tracking=True
    od.setup()
    assert od.get('motion_per_cm') is not None and od.get('motion_per_rad') is not None

    # rew = PiRewardCircuit(conn, 'SERIAL_PORT_ON_PI')  # TODO serial port
    # TODO lever

    # setup game
    doom = DOOM('doom/scenarios/arena_lowered.wad', 'map01')
    game_over = False

    while not game_over:

        # run VR devices
        od.loop()
        smooth_flo.loop()

        # action
        movement = smooth_flo.get_vel()
        if np.any(movement > 0):
            print('mov:', movement)

        # movement[:2] /= od.get('motion_per_cm')  # TODO !made it too slow? also in doom.step()
        # movement[2] /= od.get('motion_per_rad')  # TODO !made it too slow? also in doom.step()
        action = (movement, 0)  # TODO lever

        # step
        state, reward, terminated, truncated, info = doom.step(action)
        game_over = terminated or truncated
        pm.loop(pos=np.array([state.position_x, state.position_y]), vel=np.array([state.velocity_x, state.velocity_y]))

        # TODO process state
        # TODO dispense reward
        # TODO train rat

    # cleanup happens on device
