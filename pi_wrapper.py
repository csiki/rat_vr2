import socket
import types
import inspect
import time
import pickle
from typing import Callable, Any, Union, Tuple

from motion import MotionSensor, MotionSensors, SmoothMotion


# wrap functions present on the raspberry pi:
#   omni drive, motion sensors, reward, speakers, ultrasound, air puffs


class PiOverSocket:
    def __init__(self, conn_sock: socket.socket, base_cls):
        # not calling super init; only a wrapper
        self.conn_sock = conn_sock
        self.base_cls = base_cls
        self.funs = inspect.getmembers(base_cls, predicate=inspect.isfunction)
        self.host_id = id(self)  # used on raspberry to establish correspondence between wrapped and real objects

        for fun_name, fun in self.funs:
            f = self._get_fun(fun)
            f.__name__ = fun_name
            setattr(self, fun_name, f)

    def _get_fun(self, fun):
        # necessary: https://stackoverflow.com/questions/13079299/dynamically-adding-methods-to-a-class
        return lambda *args_, **kwargs_: self._send_cmd(fun, *args_, **kwargs_)

    def _send_cmd(self, fun: Callable, *args, **kwargs):
        cmd = {'o': id(self), 'c': self.base_cls.__name__, 'f': fun.__name__, 'args': args, 'kwargs': kwargs}
        print(cmd)  # TODO test
        self.conn_sock.sendall(pickle.dumps(cmd))

        pi_ret = self.conn_sock.recv(2048)
        ret_val, exception = pickle.loads(pi_ret)
        if exception:
            raise exception
        return ret_val


class PiMotionSensor(MotionSensor, PiOverSocket):
    def __init__(self, conn_sock: socket.socket, *args, **kwargs):
        PiOverSocket.__init__(self, conn_sock, self.__class__.__base__)
        getattr(self, '__init__')(*args, **kwargs)


# TODO rest of the wrappers, same as PiMotionSensor


if __name__ == '__main__':
    # p = PiMotionSensor(None, 0, 'front')
    # p.get_rel_motion(get_dt=True)
    # exit()

    host = '127.0.0.1'  # rbpi: '192.168.0.108'
    port = 4444

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.bind((host, port))
        s.listen()

        conn, addr = s.accept()

        with conn:
            print(f"Connected by {addr}")
            p = PiMotionSensor(conn, 0, 'front')
            for i in range(100):
                print(p.get_rel_motion())
                time.sleep(1)
