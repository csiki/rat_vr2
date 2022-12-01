import socket
import subprocess
import types
import inspect
import time
import pickle
import sched
from typing import Callable, Any, Union, Tuple
from pathlib import Path

import numpy as np

from motion import MotionSensor, MotionSensors, SmoothMotion
from actuator import LinActuator
from omni_drive import OmniDrive
from player_movement import PlayerMovement, Feedback
from reward import RewardCircuit


# wrap classes/functions for objects present on the raspberry pi:
#   omni drive, motion sensors, reward (feeder, air puffs), ultrasound, lever, audio

class PiCallback(Callable):
    def __init__(self, f: Callable):
        self._wrap_id = id(f)
        self._called = False

    def __call__(self, *args, **kwargs):
        self._called = True


class PiOverSocket:

    CALLBACKS = {}

    def __init__(self, _conn_sock: socket.socket, _base_cls: type):
        # not calling super init; only a wrapper
        self._conn_sock = _conn_sock
        self._base_cls = _base_cls
        self._wrap_id = id(self)  # used on raspberry to establish correspondence between wrapped and real objects
        self._real_id = None  # once the real dev obj is instantiated remotely, this contains its remote id

        # add functions to wrapper
        funs = inspect.getmembers(_base_cls, predicate=inspect.isfunction)
        for fun_name, fun in funs:
            if fun_name not in ('_get_fun', '_send_cmd'):
                f = self._get_fun(fun)
                f.__name__ = fun_name
                setattr(self, fun_name, f)
                # TODO if loop is slow, could create async version of each function, naming them f'async_{fun_name}'
                #   + implement below _async_send_cm() and _async_get_fun() AND add them above to prohibited funs above

        # get and set members functions, call the __getattribute__ and __setattr__ on the other side
        g = self._get_fun(self.__getattribute__)
        g.__name__ = '__getattribute__'
        setattr(self, 'get', g)

        s = self._get_fun(self.__setattr__)
        s.__name__ = '__setattr__'
        setattr(self, 'set', s)

    def get(self, field):  # overwritten in __init__
        return None

    def set(self, field, val):  # overwritten in __init__
        pass

    def __getstate__(self):
        return self._base_cls, self._wrap_id, self._real_id

    def __setstate__(self, state):
        self._base_cls, self._wrap_id, self._real_id = state

    def _get_fun(self, fun):
        #  _get_fun() is necessary: https://stackoverflow.com/questions/13079299/dynamically-adding-methods-to-a-class
        return lambda *args_, **kwargs_: self._send_cmd(fun, *args_, **kwargs_)

    def _send_cmd(self, fun: Callable, *args, **kwargs):

        # if any of the arguments is a function (callable), then add id and store it for a later call
        def _wrap_arg(a):
            if isinstance(a, PiOverSocket):
                return a  # nothing to do, receiver handles everything
            elif callable(a):
                wrap_a = PiCallback(a)  # wrap into special callback object to pickle and track if its called
                PiOverSocket.CALLBACKS[wrap_a._wrap_id] = a  # storing the original to call when needed
                return wrap_a
            return a

        args = [_wrap_arg(a) for a in args]
        kwargs = {k: _wrap_arg(a) for k, a in kwargs.items()}

        cmd = {'o': id(self), 'c': self._base_cls.__name__, 'f': fun.__name__, 'a': args, 'kwa': kwargs}
        # print('->', cmd)
        self._conn_sock.sendall(pickle.dumps(cmd))

        pi_ret = self._conn_sock.recv(4096)
        # print('<-', len(pi_ret), pi_ret)
        if len(pi_ret) == 0:
            raise ConnectionError('Pi disconnected')
        ret_val, exception, callback_ids_to_run = pickle.loads(pi_ret)

        for cbid in callback_ids_to_run:  # run them
            PiOverSocket.CALLBACKS.pop(cbid)()

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


class PiRewardCircuit(RewardCircuit, PiOverSocket):
    def __init__(self, _conn_sock: socket.socket, *args, **kwargs):
        PiOverSocket.__init__(self, _conn_sock, self.__class__.__base__)
        self.real_id = getattr(self, '__init__')(*args, **kwargs)


class Win2PiAudio:
    # from: https://raspberrypi.stackexchange.com/a/11744/149645

    def __init__(self, pi_ip, pi_user='rpdpi', pi_pass='666666', linco_path=''):
        self.pi_ip = pi_ip
        self.pi_user = pi_user
        self.pi_pass = pi_pass  # i know, change the default if you are scared
        self.linco_path = Path(linco_path)  # leave default if linco.exe is added to the Path environment var
        self.proc = None

    def start(self):
        cmd = [str(self.linco_path / 'linco.exe'), '-B', '16', '-C', '2', '-R', '44100',
               '|', 'plink', self.pi_ip, '-l', self.pi_user, '-pw', self.pi_pass,
               'cat - | pacat --server 127.0.0.1 --playback']
        print('run:', ' '.join(cmd))
        self.proc = subprocess.Popen(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=False)

    def is_running(self):
        return self.proc.poll() is None if self.proc else False

    def cleanup(self):
        self.proc.terminate()


class ServerSocket:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock: socket.socket = None
        self.conn: socket.socket = None
        self.conn_addr = None

    def __enter__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen()

        print(f'Accepting connections on {self.host}:{self.port}')
        self.conn, self.conn_addr = self.sock.accept()
        self.conn.settimeout(10)  # waiting on blocking calls
        self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        return self.conn

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.conn:
            self.conn.close()
        if self.sock:
            self.sock.close()


def test_wrapper():
    # pc/server address
    host = socket.gethostbyname(socket.gethostname())
    port = 4444

    # omni drive
    lin_act_pins = {'up': 22, 'down': 4, 'enable': 27}
    roller0_pins = {'right': 23, 'left': 24, 'pwm': 18}
    roller1_pins = {'right': 5, 'left': 6, 'pwm': 13}
    roller2_pins = {'right': 25, 'left': 26, 'pwm': 12}
    roller_pins = [roller0_pins, roller1_pins, roller2_pins]

    with ServerSocket(host, port) as conn:

        p = PiMotionSensor(conn, 0, 1, 0)
        ps = PiSmoothMotion(conn, p, 0.1)

        p2 = PiMotionSensor(conn, 1, 0, 1)
        ps2 = PiSmoothMotion(conn, p2, 0.1)
        od = PiOmniDrive(conn, roller_pins, lin_act_pins)
        od.setup()

        last_od = time.time()
        forward = True
        loop_delay = .5
        loop_times = []

        for i in range(200):
            loop_start = time.time()

            if time.time() - last_od > 6:
                # od.simple_drive('right_turn' if forward else 'left_turn', .7, t=4)
                od.simple_drive('forward' if forward else 'backward', .7, t=2.,
                                callback=lambda: od.simple_drive('right_turn', .7, t=2))
                forward = not forward
                last_od = time.time()

            p.loop()
            p2.loop()

            od.loop()
            print('-' * 80)

            print(p.get_vel(), '- smooth -> ', ps.get_vel())
            print(p2.get_vel(), '- smooth -> ', ps2.get_vel())

            loop_times.append(time.time() - loop_start)
            time.sleep(loop_delay)

        print('avg loop time:', np.mean(loop_times), np.std(loop_times))
        print(loop_times)


def just_read_motion():
    # pc/server address
    host = socket.gethostbyname(socket.gethostname())
    port = 4444
    print_delay = .8

    with ServerSocket(host, port) as conn:
        flo1 = PiMotionSensor(conn, 0, 1, 0, invert_x=True, invert_y=True, swap_xy=True)
        flo2 = PiMotionSensor(conn, 1, 0, 1, invert_x=False, invert_y=True, swap_xy=True)
        flo = PiMotionSensors(conn, flo1, flo2)
        smooth_flo = PiSmoothMotion(conn, flo, 0.1)

        last_print = time.time()
        ts = []

        while True:
            s = time.time()

            smooth_flo.loop()
            if time.time() - last_print > print_delay:
                print('-' * 80)
                # print('1', flo1.get_vel())
                # print('2', flo2.get_vel())
                print('c', smooth_flo.get_vel())
                last_print = time.time()
            # time.sleep(0.01)

            ts.append(time.time() - s)
            if len(ts) % 200 == 0:
                print('T:', np.mean(ts))


if __name__ == '__main__':
    # test_wrapper()
    just_read_motion()
