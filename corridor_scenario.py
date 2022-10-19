import socket
import time
import traceback
import numpy as np

from DOOM import DOOM
from motion import MotionSensor, MotionSensors, SmoothMotion
from actuator import LinActuator
from omni_drive import OmniDrive
from player_movement import PlayerMovement, Feedback

from pi_wrapper import PiSmoothMotion, PiMotionSensor, PiMotionSensors, PiOmniDrive, PiFeedback, PiPlayerMovement, ServerSocket


# pc/server address
host, port = '192.168.0.129', 4444  # '127.0.0.1'


with ServerSocket(host, port) as conn:

    # setup VR
    flo1 = PiMotionSensor(conn, 0, 1, 0, invert_x=True, invert_y=True, swap_xy=True)
    flo2 = PiMotionSensor(conn, 1, 0, 1, invert_x=False, invert_y=True, swap_xy=True)
    flo = PiMotionSensors(conn, flo1, flo2)
    smooth_flo = PiSmoothMotion(conn, flo, 0.1)

    pm = PlayerMovement(do_calc_acc=True)

    lin_act_pins = {'up': 22, 'down': 4, 'enable': 27}
    roller0_pins = {'right': 23, 'left': 24, 'pwm': 18}
    roller1_pins = {'right': 5, 'left': 6, 'pwm': 13}
    roller2_pins = {'right': 25, 'left': 26, 'pwm': 12}
    roller_pins = [roller0_pins, roller1_pins, roller2_pins]
    od = PiOmniDrive(conn, roller_pins, lin_act_pins, mount_tracking=True)  # TODO calibration first, then pass it in here
    assert od.motion_per_cm is not None and od.motion_per_rad is not None

    # todo lever, reward

    # setup game
    doom = DOOM('../rat_vr/doom/scenarios/corridor_straight.wad', 'map01')
    game_over = False

    while not game_over:

        # run VR devices
        od.loop()
        flo.loop()

        # action
        movement = smooth_flo.get_vel()
        movement[:2] /= od.motion_per_cm
        movement[2] /= od.motion_per_rad
        action = (movement, 0)  # TODO lever

        # step
        state, reward, terminated, truncated, info = doom.step(action)
        game_over = terminated or truncated
        pm.loop(pos=np.array([state.position_x, state.position_y]), vel=np.array([state.velocity_x, state.velocity_y]))

        # TODO process state
        # TODO dispense reward
        # TODO train rat

    # cleanup happens on device
