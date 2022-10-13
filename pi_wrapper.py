import socket
import types
import inspect
import time
import pickle
import sched
from typing import Callable, Any, Union, Tuple

from motion import MotionSensor, MotionSensors, SmoothMotion


# wrap classes/functions present on the raspberry pi:
#   omni drive, motion sensors, reward (feeder, air puffs), ultrasound, lever, (speakers)


class PiOverSocket:
    def __init__(self, conn_sock: socket.socket, base_cls):
        # not calling super init; only a wrapper
        self.conn_sock = conn_sock
        self.base_cls = base_cls
        self.funs = inspect.getmembers(base_cls, predicate=inspect.isfunction)
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
        print('->', cmd)
        self.conn_sock.sendall(pickle.dumps(cmd))

        pi_ret = self.conn_sock.recv(4096)
        print('<-', len(pi_ret), pi_ret)
        if len(pi_ret) == 0:
            raise ConnectionError('Pi disconnected')
        ret_val, exception = pickle.loads(pi_ret)
        if exception:
            raise exception from exception
        return ret_val


class PiMotionSensor(MotionSensor, PiOverSocket):  # first base needs to be the wrapped device class
    def __init__(self, conn_sock: socket.socket, *args, **kwargs):
        PiOverSocket.__init__(self, conn_sock, self.__class__.__base__)  # init wrapper
        # no need to init the wrapped class, the instantiated object here only serves as a wrapper to member functions
        self.real_id = getattr(self, '__init__')(*args, **kwargs)  # calling init remotely, returns remote id


class PiSmoothMotion(SmoothMotion, PiOverSocket):  # first base needs to be the wrapped device class
    def __init__(self, conn_sock: socket.socket, *args, **kwargs):
        PiOverSocket.__init__(self, conn_sock, self.__class__.__base__)  # init wrapper
        # no need to init the wrapped class, the instantiated object here only serves as a wrapper to member functions
        self.real_id = getattr(self, '__init__')(*args, **kwargs)  # calling init remotely, returns remote id


# TODO rest of the wrappers, same as PiMotionSensor


if __name__ == '__main__':

    # pc/server address
    host, port = '192.168.0.129', 4444  # '127.0.0.1'

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # TODO
        sock.bind((host, port))
        sock.listen()

        print('Accepting connections..')
        conn, addr = sock.accept()

        with conn:
            print(f"Connected by {addr}")
            p = PiMotionSensor(conn, 0, 'front')
            ps = PiSmoothMotion(conn, p, 2)

            for i in range(100):
                p.loop()
                print(p.get_vel(), '- smooth -> ', print(ps.get_vel()))  # TODO ps returns None
                time.sleep(1)
