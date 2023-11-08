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
from trainer import ArenaTrainer, ManualTrainer
from DOOM import DOOM
from vizdoom import ScreenResolution


class LiveLinePlot:
    # https://matplotlib.org/stable/tutorials/advanced/blitting.html

    def __init__(self, nplots=1, update_freq=1, xlim=(-10, 0), ylim=(-1, 1),
                 title='', xlabel='', ylabel='', active=True, max_sampling_freq=100):
        self.update_freq = update_freq
        self.active = active
        self.nplots = nplots
        self.xlim = xlim
        self.ylim = ylim
        self.xlabel = xlabel
        self.ylabel = ylabel
        self.title = title

        self.it = np.random.randint(0, 100)  # live plots will not as likely be updated in sync if update_freq > 1

        assert xlim[0] < 0  # x is time in the past within (-inf, 0]

        self.xlim = xlim
        self.ylim = ylim
        max_sample = int(max_sampling_freq * np.ptp(xlim))
        self.xdata, self.ydata = deque(maxlen=max_sample), [deque(maxlen=max_sample) for _ in range(nplots)]

        if active:
            self._init_fig()

    def _init_fig(self):
        self.fig, self.ax = plt.subplots(1, 1)
        cols = [c for c in matplotlib.colors.ColorConverter.colors.keys() if len(c) == 1]
        self.lns = [self.ax.plot([], [], f'{cols[i]}-', animated=True, alpha=.7, label=f'{i}')[0]
                    for i in range(self.nplots)]
        self.ax.set_xlim(*self.xlim)
        self.ax.set_ylim(*self.ylim)
        self.ax.set_xlabel(self.xlabel)
        self.ax.set_ylabel(self.ylabel)
        plt.legend()
        self.ax.set_title(self.title)
        self.ax.spines[['right', 'top']].set_visible(False)
        self.ax.grid(linestyle='--', linewidth=0.5)

        self.fig.set_visible(self.active)
        plt.show(block=False)
        plt.pause(0.1)

        self.bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        for ln in self.lns:
            self.ax.draw_artist(ln)
        self.fig.canvas.blit(self.fig.bbox)

    def activate(self, is_active=True):
        self.active = is_active
        if is_active:
            self._init_fig()

    def update(self, x, ys):
        if not self.active:
            return

        self.xdata.append(x)
        for i, y in enumerate(ys):
            self.ydata[i].append(y)

        self.it += 1
        if self.it % self.update_freq != 0:
            return

        # if self.it % 100 == 0:
        #     xdata = np.asarray(self.xdata)
        #     xdata -= xdata[-1]  # last record to front
        #
        #     too_far_in_the_past = np.cumsum(np.asarray(self.xdata)[::-1])[::-1] < self.xlim[-1] * 2
        #     for _ in range(too_far_in_the_past.sum()):
        #         self.xdata.popleft()
        #         for y in self.ydata:
        #             y.popleft()

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
    reward_serial_port = '/dev/ttyACM0'

    with ServerSocket(host, port) as conn:

        # setup VR
        flo1 = PiMotionSensor(conn, **FRONT_MOTION_PARAMS)
        flo2 = PiMotionSensor(conn, **SIDE_MOTION_PARAMS)
        flo = PiMotionSensors(conn, flo1, flo2)

        # smooth_flo1 = PiSmoothMotion(conn, flo1, 0.1)
        # smooth_flo2 = PiSmoothMotion(conn, flo2, 0.1)
        smooth_flo = PiSmoothMotion(conn, flo, 0.1, 100)

        pm = PlayerMovement(smooth_dt=.1)

        calibration_path = 'omni_calib.pckl'
        od = PiOmniDrive(conn, auto_mounting=False, mount_init=False, calib_path=calibration_path)
        od.setup()
        assert od.get('motion_per_cm') is not None and od.get('motion_per_rad') is not None

        reward_circuit = PiRewardCircuit(conn, reward_serial_port, auto_mixing_at_every=10)

        # setup live plots
        # mov_1_lp = LiveLinePlot(nplots=2, ylim=(-1500, 1500), title='mov1')
        # mov_2_lp = LiveLinePlot(nplots=2, ylim=(-1500, 1500), title='mov2')
        smooth_mov_lp = LiveLinePlot(nplots=3, update_freq=4, ylim=(-1200, 1200), title='smooth mov', active=True)
        phys_mov_lp = LiveLinePlot(nplots=3, update_freq=4, ylim=(-15, 15), title='phys mov', active=True)
        ingame_mov_lp = LiveLinePlot(nplots=3, update_freq=4, ylim=(-15, 15), title='ingame mov', active=False)
        player_pos_lp = LiveLinePlot(nplots=3, update_freq=4, ylim=(-1000, 1000), title='player pos', active=False)
        player_vel_lp = LiveLinePlot(nplots=3, update_freq=4, ylim=(-15, 15), title='player vel', active=False)
        player_acc_lp = LiveLinePlot(nplots=3, update_freq=4, ylim=(-15, 15), title='player acc', active=False)

        # setup game
        player_mode = vizdoom.Mode.PLAYER  # vizdoom.Mode.SPECTATOR | vizdoom.Mode.PLAYER
        cfg_update = {'is_async': False, 'fullscreen': False, 'res': ScreenResolution.RES_640X480, 'mode': player_mode}
        doom = DOOM('doom/scenarios/arena_lowered.wad', 'map01', cfg_update)
        game_over = False

        # trainer = ArenaTrainer(cspace_path='arena_lowered.map01.pckl', omni_drive=od)  # TODO provide player_movement
        # artificial_train_mov = .5  # TODO 1. means movement is defined by omnidrive roll goal, not the sensed motion
        trainer = ManualTrainer(doom, od, reward_circuit, move_r_per_sec=2, kill_r=100,
                                r_in_every=.8, min_r_given=50, omni_speed=.75)

        while not game_over:

            # run VR devices
            od.loop()
            smooth_flo.loop()
            rc_state = reward_circuit.loop()

            # smooth_flo1.loop()
            # smooth_flo2.loop()

            # action
            # mov = np.array([0, 0, 0])
            mov = smooth_flo.get_vel()
            # mov1 = smooth_flo1.get_vel()
            # mov2 = smooth_flo2.get_vel()

            # phys_mov = mov
            lever = 0 < rc_state['LEV'] < 800 if rc_state is not None else 0
            phys_mov = od.motion_to_phys(mov)
            action = (phys_mov, lever)
            # TODO get current aim from omni drive if any, then mix it up with phys_mov:
            #   weight the 2 according to artificial_train_mov;
            #   take the magnitude of phys_mov and translate od.current_drive_v to the same magnitude before weighting

            # step
            state, reward, terminated, truncated, info = doom.step(action)
            game_over = terminated or truncated
            # pm.loop(pos=np.array([state.position_x, state.position_y, state.angle]),
            #         vel=np.array([state.velocity_x, state.velocity_y]))  # TODO

            # update plots
            t = time.time()
            # mov_1_lp.update(t, mov1)
            # mov_2_lp.update(t, mov2)

            trainer.enforce_action(info['step_i'], state)
            reward += trainer.give_reward(info['step_i'], state)

            smooth_mov_lp.update(t, mov)
            phys_mov_lp.update(t, phys_mov * [1, 1, 180/np.pi])
            ingame_mov_lp.update(t, info['action'][:3])
            player_pos_lp.update(t, pm.pos)
            player_vel_lp.update(t, pm.vel)
            player_acc_lp.update(t, pm.acc)

        # cleanup happens on device automatically
