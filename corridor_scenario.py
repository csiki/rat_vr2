import socket
import time

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
    smooth_flo1 = PiSmoothMotion(conn, flo1, 0.1)
    smooth_flo2 = PiSmoothMotion(conn, flo2, 0.1)
    flo = PiMotionSensors(conn, flo1, flo2)
    smooth_flo = PiSmoothMotion(conn, flo, 0.1)

    # setup game
    # doom = DOOM('../rat_vr/doom/scenarios/corridor_straight.wad', 'map01')
    game_over = False

    while not game_over:

        # get action
        flo.loop()
        print('-'*80)
        print('flo1', flo1.get_vel(), '- smooth -> ', smooth_flo1.get_vel())
        print('flo2', flo2.get_vel(), '- smooth -> ', smooth_flo2.get_vel())
        print('comb', flo.get_vel(), '- smooth -> ', smooth_flo.get_vel())
        # print('- smooth -> ', smooth_flo.get_vel())
        time.sleep(.5)



        # state, reward, terminated, truncated, info = doom.step(a)
        # game_over = terminated or truncated

