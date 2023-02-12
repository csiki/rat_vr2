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


if __name__ == '__main__':

    # pc/server address
    host, port = '192.168.0.129', 4444  # TODO as cmd argument
    serial_port = '/dev/ttyACM0'

    with ServerSocket(host, port) as conn:
        reward_circuit = PiRewardCircuit(conn, serial_port)
        reward_circuit.send_cmd(left_blow_ms=4000, right_blow_ms=3000)
        time.sleep(2)
        reward_circuit.stop()
        time.sleep(2)
