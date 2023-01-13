import time
import numpy as np
from datetime import datetime
import queue
import asyncio
from collections import deque

import matplotlib
import matplotlib.pyplot as plt
import vizdoom
from matplotlib.animation import FuncAnimation

import socket
import traceback

import matplotlib.pyplot as plt
import numpy as np

from player_movement import PlayerMovement, Feedback
from config import *

from pi_wrapper import PiSmoothMotion, PiMotionSensor, PiMotionSensors, PiOmniDrive, PiFeedback, \
    PiPlayerMovement, ServerSocket, PiRewardCircuit
from trainer import ArenaTrainer
from DOOM import DOOM
from vizdoom import ScreenResolution


class LiveLinePlot:
    # https://matplotlib.org/stable/tutorials/advanced/blitting.html

    def __init__(self, nplots=1, update_freq=1, xlim=(-10, 0), ylim=(-1, 1), title='', xlabel='', ylabel=''):
        self.update_freq = update_freq
        self.it = np.random.randint(0, 100)  # live plots will not as likely be updated in sync if update_freq > 1

        assert xlim[0] < 0  # x is time in the past within (-inf, 0]

        self.xlim = xlim
        self.ylim = ylim
        self.xdata, self.ydata = deque(), [deque() for _ in range(nplots)]

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

        self.it += 1
        if self.it % self.update_freq != 0:
            return

        if self.it % 100 == 0:
            xdata = np.asarray(self.xdata)
            xdata -= xdata[-1]  # last record to front

            too_far_in_the_past = np.cumsum(np.asarray(self.xdata)[::-1])[::-1] < self.xlim[-1] * 2
            for _ in range(too_far_in_the_past.sum()):
                self.xdata.popleft()
                for y in self.ydata:
                    y.popleft()

        xdata = np.asarray(self.xdata)
        xdata -= xdata[-1]  # last record to front

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

    with ServerSocket(host, port) as conn:

        # setup VR
        flo1 = PiMotionSensor(conn, **FRONT_MOTION_PARAMS)
        flo2 = PiMotionSensor(conn, **SIDE_MOTION_PARAMS)
        flo = PiMotionSensors(conn, flo1, flo2)

        # smooth_flo1 = PiSmoothMotion(conn, flo1, 0.1)
        # smooth_flo2 = PiSmoothMotion(conn, flo2, 0.1)
        smooth_flo = PiSmoothMotion(conn, flo, 0.1, 100)

        pm = PlayerMovement(smooth_dt=.3)

        calibration_path = 'omni_calib.pckl'
        od = PiOmniDrive(conn, mount_tracking=False, calib_path=calibration_path)  # TODO mount_tracking=True
        od.setup()
        assert od.get('motion_per_cm') is not None and od.get('motion_per_rad') is not None

        trainer = ArenaTrainer(cspace_path='arena_lowered.map01.pckl', omni_drive=od)

        # rew = PiRewardCircuit(conn, 'SERIAL_PORT_ON_PI')  # TODO serial port
        # TODO lever

        # setup live plots
        # mov_1_lp = LiveLinePlot(nplots=2, ylim=(-1500, 1500), title='mov1')
        # mov_2_lp = LiveLinePlot(nplots=2, ylim=(-1500, 1500), title='mov2')
        smooth_mov_lp = LiveLinePlot(nplots=3, update_freq=4, ylim=(-1200, 1200), title='smooth mov')
        phys_mov_lp = LiveLinePlot(nplots=3, update_freq=4, ylim=(-15, 15), title='phys mov')
        ingame_mov_lp = LiveLinePlot(nplots=3, update_freq=4, ylim=(-15, 15), title='ingame mov')
        player_pos_lp = LiveLinePlot(nplots=3, update_freq=4, ylim=(-1000, 1000), title='player pos')
        player_vel_lp = LiveLinePlot(nplots=3, update_freq=4, ylim=(-15, 15), title='player vel')
        player_acc_lp = LiveLinePlot(nplots=3, update_freq=4, ylim=(-15, 15), title='player acc')

        # setup game
        cfg_update = {'fullscreen': False, 'res': ScreenResolution.RES_640X480, 'mode': vizdoom.Mode.SPECTATOR}
        doom = DOOM('doom/scenarios/arena_lowered.wad', 'map01', cfg_update)
        game_over = False

        while not game_over:

            # run VR devices
            od.loop()
            smooth_flo.loop()
            # smooth_flo1.loop()
            # smooth_flo2.loop()

            # action
            mov = smooth_flo.get_vel()
            # mov1 = smooth_flo1.get_vel()
            # mov2 = smooth_flo2.get_vel()

            phys_mov = od.motion_to_phys(mov)
            action = (phys_mov, 0)  # TODO lever

            # step
            state, reward, terminated, truncated, info = doom.step(action)
            game_over = terminated or truncated
            pm.loop(pos=np.array([state.position_x, state.position_y, state.angle]),
                    vel=np.array([state.velocity_x, state.velocity_y]))

            # update plots
            t = time.time()
            # mov_1_lp.update(t, mov1)
            # mov_2_lp.update(t, mov2)
            smooth_mov_lp.update(t, mov)
            phys_mov_lp.update(t, phys_mov)
            ingame_mov_lp.update(t, info['action'][:3])
            player_pos_lp.update(t, pm.pos)
            player_vel_lp.update(t, pm.vel)
            player_acc_lp.update(t, pm.acc)

        # cleanup happens on device automatically
