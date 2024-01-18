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

        # rc_state = reward_circuit.send(pressure_setpoint=100)

        # rc_state = reward_circuit.stat(verbose=True)
        # print('state1', rc_state)
        #
        # reward_circuit.send(press_lever_ms=1000)
        # rc_state = reward_circuit.stat(verbose=True)
        # print('state2', rc_state)
        # time.sleep(2)
        # rc_state = reward_circuit.stat(verbose=True)
        # print('state3', rc_state)
        # reward_circuit.send(press_lever_ms=1000)
        # time.sleep(2)
        # rc_state = reward_circuit.stat(verbose=True)
        # print('state4', rc_state)
        #
        # reward_circuit.send(press_lever_ms=1000)
        # time.sleep(200)
        # rc_state = reward_circuit.stat(verbose=True)
        # print('state5', rc_state)
        # time.sleep(2)
        # rc_state = reward_circuit.stat(verbose=True)
        # print('state6', rc_state)

        # for i in range(50):
        #     print('open')
        #     reward_circuit.send(valve_open_ms=2000)#, pressure_setpoint=0)
        #     time.sleep(2000 / 1000)
        #     print('close')
        #     reward_circuit.send(valve_open_ms=0)#, pressure_setpoint=0)#, pressure_setpoint=0)
        #     time.sleep(2000 / 1000)
        #     print(i)

        # reward_circuit.send(valve_open_ms=3000, left_blow_ms=2000)
        # time.sleep(4)
        # reward_circuit.send(valve_open_ms=3000, left_blow_ms=2000)
        # time.sleep(4)
        # reward_circuit.send(valve_open_ms=3000, left_blow_ms=2000)
        # time.sleep(6)
        # reward_circuit.stop()

        reward_circuit.update(left_blow_ms=5000)
        reward_circuit.loop()
        time.sleep(1)
        reward_circuit.update(left_blow_ms=1)
        reward_circuit.loop()
        print('a')
        time.sleep(10)

        # reward_circuit.send(left_blow_ms=10000, right_blow_ms=2000)
        # time.sleep(10)
        reward_circuit.send(left_blow_ms=8000)
        time.sleep(8)
        reward_circuit.send(right_blow_ms=8000)
        time.sleep(8)

        reward_circuit.stop()
        time.sleep(2)
