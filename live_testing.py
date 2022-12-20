import time
import numpy as np
from datetime import datetime
import queue
import asyncio

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import socket
import traceback

import matplotlib.pyplot as plt
import numpy as np

from player_movement import PlayerMovement, Feedback
from config import *

from pi_wrapper import PiSmoothMotion, PiMotionSensor, PiMotionSensors, PiOmniDrive, PiFeedback, \
    PiPlayerMovement, ServerSocket, PiRewardCircuit
from DOOM import DOOM


class LiveLinePlot:
    # https://matplotlib.org/stable/tutorials/advanced/blitting.html

    def __init__(self, nplots=1, xlim=(-10, 0), ylim=(-1, 1), title='', xlabel='', ylabel=''):
        self.xlim = xlim
        self.ylim = ylim
        self.xdata, self.ydata = [], [[] for _ in range(nplots)]

        self.fig, self.ax = plt.subplots(1, 1)
        cols = [c for c in matplotlib.colors.ColorConverter.colors.keys() if len(c) == 1]
        self.lns = [self.ax.plot([], [], f'{cols[i]}-', animated=True, alpha=.7, label=f'{i}')[0]
                    for i in range(nplots)]
        self.ax.set_xlim(*xlim)
        self.ax.set_ylim(*ylim)
        self.ax.set_xlabel(xlabel)
        self.ax.set_ylabel(ylabel)
        plt.legend()
        self.ax.set_title(title)
        plt.show(block=False)
        plt.pause(0.1)

        self.bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        for ln in self.lns:
            self.ax.draw_artist(ln)
        self.fig.canvas.blit(self.fig.bbox)

    def update(self, x, ys):
        self.xdata.append(x)
        for i, y in enumerate(ys):
            self.ydata[i].append(y)

        xdata = np.asarray(self.xdata)
        xdata -= xdata[-1]

        self.fig.canvas.restore_region(self.bg)
        for i, ln in enumerate(self.lns):
            ln.set_xdata(xdata)
            ln.set_ydata(self.ydata[i])
            self.ax.draw_artist(ln)

        self.fig.canvas.blit(self.fig.bbox)
        self.fig.canvas.flush_events()


if __name__ == '__main__':

    # pc/server address
    host, port = '192.168.0.129', 4444  # TODO as cmd argument

    # TODO run doom in no window mode

    with ServerSocket(host, port) as conn:

        # setup VR
        flo1 = PiMotionSensor(conn, **FRONT_MOTION_PARAMS)
        flo2 = PiMotionSensor(conn, **SIDE_MOTION_PARAMS)
        flo = PiMotionSensors(conn, flo1, flo2)

        smooth_flo1 = PiSmoothMotion(conn, flo1, 0.1)
        smooth_flo2 = PiSmoothMotion(conn, flo2, 0.1)
        smooth_flo = PiSmoothMotion(conn, flo, 0.1)

        pm = PlayerMovement(do_calc_acc=True)

        calibration_path = 'omni_calib.pckl'
        od = PiOmniDrive(conn, mount_tracking=False, calib_path=calibration_path)  # TODO mount_tracking=True
        od.setup()
        assert od.get('motion_per_cm') is not None and od.get('motion_per_rad') is not None

        # rew = PiRewardCircuit(conn, 'SERIAL_PORT_ON_PI')  # TODO serial port
        # TODO lever

        # setup game
        # doom = DOOM('doom/scenarios/arena_lowered.wad', 'map01')
        game_over = False

        mov_1_lp = LiveLinePlot(nplots=2, ylim=(-1500, 1500), title='mov1')
        mov_2_lp = LiveLinePlot(nplots=2, ylim=(-1500, 1500), title='mov2')
        smooth_mov_lp = LiveLinePlot(nplots=3, ylim=(-1200, 1200), title='smooth')
        phys_mov_lp = LiveLinePlot(nplots=3, ylim=(-100, 100), title='phys')
        # TODO track pm as well

        while not game_over:

            # run VR devices
            od.loop()
            smooth_flo.loop()
            smooth_flo1.loop()
            smooth_flo2.loop()

            # action
            movement = smooth_flo.get_vel()
            mov1 = smooth_flo1.get_vel()
            mov2 = smooth_flo2.get_vel()

            # plots
            mov_1_lp.update(time.time(), mov1)
            mov_2_lp.update(time.time(), mov2)
            smooth_mov_lp.update(time.time(), movement)

            movement = od.motion_to_phys(movement)
            phys_mov_lp.update(time.time(), movement)
            # action = (movement, 0)  # TODO lever

            # # step
            # state, reward, terminated, truncated, info = doom.step(action)
            # game_over = terminated or truncated
            # pm.loop(pos=np.array([state.position_x, state.position_y]),
            #         vel=np.array([state.velocity_x, state.velocity_y]))

        # cleanup happens on device automatically
