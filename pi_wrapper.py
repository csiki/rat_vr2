import socket
import types
import inspect
import time
import pickle
import sched
from typing import Callable, Any, Union, Tuple

import numpy as np

from motion import MotionSensor, MotionSensors, SmoothMotion
from actuator import LinActuator
from omni_drive import OmniDrive
from player_movement import PlayerMovement, Feedback


# wrap classes/functions present on the raspberry pi:
#   omni drive, motion sensors, reward (feeder, air puffs), ultrasound, lever, (speakers)


# TODO callback functions won't work through the socket right now; how to solve it:
#   create a Callback object
#   wrapt it: PiCallback(conn, Callback)
#     wrapping it overwrites the __call__ function to registering the wrapped callback to a global space
#       be that Callbacks.has_run[] on the Pi side
#   pass it to Pi* device cls function
#   when the callback is called on Pi: add callback to Callbacks.has_run
#   at the next response to pc, take all has_run callbacks, return their ids, and call them on pc
#   pc: Callbacks.registered{} contains id->Callback callbacks created, but has not run
#   pi: PiCallbacks.registered{} has the same


class PiOverSocket:
    def __init__(self, _conn_sock: socket.socket, _base_cls: type):
        # not calling super init; only a wrapper
        self.conn_sock = _conn_sock
        self.base_cls = _base_cls
        self.funs = inspect.getmembers(_base_cls, predicate=inspect.isfunction)
        self.wrap_id = id(self)  # used on raspberry to establish correspondence between wrapped and real objects
        self.real_id = None  # once the real dev obj is instantiated remotely, this contains its remote id

        for fun_name, fun in self.funs:
            if fun_name not in ('_get_fun', '_send_cmd'):
                f = self._get_fun(fun)
                f.__name__ = fun_name
                setattr(self, fun_name, f)
                # TODO if loop is slow, could create async version of each function, naming them f'async_{fun_name}'
                #   + implement below _async_send_cm() and _async_get_fun() AND add them above to prohibited funs above

    def __getstate__(self):
        return self.base_cls, self.wrap_id, self.real_id

    def __setstate__(self, state):
        self.base_cls, self.wrap_id, self.real_id = state

    def _get_fun(self, fun):
        # necessary: https://stackoverflow.com/questions/13079299/dynamically-adding-methods-to-a-class
        return lambda *args_, **kwargs_: self._send_cmd(fun, *args_, **kwargs_)

    def _send_cmd(self, fun: Callable, *args, **kwargs):
        cmd = {'o': id(self), 'c': self.base_cls.__name__, 'f': fun.__name__, 'a': args, 'kwa': kwargs}
        # print('->', cmd)
        self.conn_sock.sendall(pickle.dumps(cmd))

        pi_ret = self.conn_sock.recv(4096)
        # print('<-', len(pi_ret), pi_ret)
        if len(pi_ret) == 0:
            raise ConnectionError('Pi disconnected')
        ret_val, exception = pickle.loads(pi_ret)
        # print('a:', ret_val, exception)
        if exception:
            raise exception from exception
        return ret_val


class PiMotionSensor(MotionSensor, PiOverSocket):  # first base needs to be the wrapped device class
    def __init__(self, _conn_sock: socket.socket, *args, **kwargs):
        PiOverSocket.__init__(self, _conn_sock, self.__class__.__base__)  # init wrapper
        # no need to init the wrapped class, the instantiated object here only serves as a wrapper to member functions
        self.real_id = getattr(self, '__init__')(*args, **kwargs)  # calling init remotely, returns remote id


class PiMotionSensors(MotionSensors, PiOverSocket):
    def __init__(self, _conn_sock: socket.socket, *args, **kwargs):
        PiOverSocket.__init__(self, _conn_sock, self.__class__.__base__)
        self.real_id = getattr(self, '__init__')(*args, **kwargs)


class PiSmoothMotion(SmoothMotion, PiOverSocket):
    def __init__(self, _conn_sock: socket.socket, *args, **kwargs):
        PiOverSocket.__init__(self, _conn_sock, self.__class__.__base__)
        self.real_id = getattr(self, '__init__')(*args, **kwargs)


class PiLinActuator(LinActuator, PiOverSocket):
    def __init__(self, _conn_sock: socket.socket, *args, **kwargs):
        PiOverSocket.__init__(self, _conn_sock, self.__class__.__base__)
        self.real_id = getattr(self, '__init__')(*args, **kwargs)


class PiOmniDrive(OmniDrive, PiOverSocket):
    def __init__(self, _conn_sock: socket.socket, *args, **kwargs):
        PiOverSocket.__init__(self, _conn_sock, self.__class__.__base__)
        self.real_id = getattr(self, '__init__')(*args, **kwargs)


class PiPlayerMovement(PlayerMovement, PiOverSocket):
    def __init__(self, _conn_sock: socket.socket, *args, **kwargs):
        PiOverSocket.__init__(self, _conn_sock, self.__class__.__base__)
        self.real_id = getattr(self, '__init__')(*args, **kwargs)


class PiFeedback(Feedback, PiOverSocket):
    def __init__(self, _conn_sock: socket.socket, *args, **kwargs):
        PiOverSocket.__init__(self, _conn_sock, self.__class__.__base__)
        self.real_id = getattr(self, '__init__')(*args, **kwargs)


if __name__ == '__main__':

    # pc/server address
    host, port = '192.168.0.129', 4444  # '127.0.0.1'

    # omni drive
    lin_act_pins = {'up': 22, 'down': 4, 'enable': 27}
    roller0_pins = {'right': 23, 'left': 24, 'pwm': 18}
    roller1_pins = {'right': 5, 'left': 6, 'pwm': 13}
    roller2_pins = {'right': 25, 'left': 26, 'pwm': 12}
    roller_pins = [roller0_pins, roller1_pins, roller2_pins]

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.bind((host, port))
        sock.listen()

        print('Accepting connections..')
        conn, addr = sock.accept()
        conn.settimeout(10)  # waiting on blocking calls
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        with conn:
            print(f"Connected by {addr}")
            p = PiMotionSensor(conn, 0, 'front')
            ps = PiSmoothMotion(conn, p, 0.1)
            od = PiOmniDrive(conn, roller_pins, lin_act_pins)
            od.setup()

            last_od = time.time()
            forward = True
            loop_delay = .2
            loop_times = []

            for i in range(200):
                loop_start = time.time()

                if time.time() - last_od > 4:
                    od.simple_drive('forward' if forward else 'backward', .7, t=1.5)
                    forward = not forward
                    last_od = time.time()

                p.loop()
                od.loop()
                # print(p.get_vel(), '- smooth -> ', ps.get_vel())
                loop_times.append(time.time() - loop_start)
                time.sleep(loop_delay)

            print('avg loop time:', np.mean(loop_times), np.std(loop_times))
            print(loop_times)
