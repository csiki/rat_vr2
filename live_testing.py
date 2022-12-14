import time

import numpy as np
from datetime import datetime
import queue
import asyncio

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class LiveLinePlot:
    # https://matplotlib.org/stable/tutorials/advanced/blitting.html

    def __init__(self, nplots=1, xlim=(-10, 0), ylim=(-1, 1)):
        self.xlim = xlim
        self.ylim = ylim
        self.xdata, self.ydata = [], [[] for _ in range(nplots)]

        self.fig, self.ax = plt.subplots(1, 1)
        cols = [c for c in matplotlib.colors.ColorConverter.colors.keys() if len(c) == 1]
        self.lns = [self.ax.plot([], [], f'{cols[i]}-', animated=True, alpha=.7, label=f'{i}')[0]
                    for i in range(nplots)]
        self.ax.set_xlim(*xlim)
        self.ax.set_ylim(*ylim)
        plt.legend()
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
