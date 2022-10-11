import types
import inspect
import pickle
from typing import Callable, Any, Union, Tuple

from motion import MotionSensor, MotionSensors, SmoothMotion


# wrap functions present on the raspberry pi:
#   omni drive, motion sensors, reward, speakers, ultrasound, air puffs

class PiMotionSensor(MotionSensor):
    def __init__(self, connector, *args, **kwargs):  # TODO ssh connector
        # not calling super init; only a wrapper
        self.connector = connector
        self.funs = inspect.getmembers(self.__class__.__base__, predicate=inspect.isfunction)

        for fun_name, fun in self.funs:
            f = self._get_fun(fun)
            f.__name__ = fun_name
            setattr(self, fun_name, f)

        # self._send_cmd(self.__init__, *args, **kwargs)
        getattr(self, '__init__')(*args, **kwargs)

    def _get_fun(self, fun):  # necessary
        return lambda *args_, **kwargs_: self._send_cmd(fun, *args_, **kwargs_)

    def _send_cmd(self, fun: Callable, *args, **kwargs):
        cmd = {'c': self.__class__.__base__.__name__, 'f': fun.__name__, 'args': args, 'kwargs': kwargs}
        print(cmd)
        # self.connector.write(pickle.dumps(cmd))  # TODO
        # TODO success, return_val = self.connector.read()
        #    return_val = pickle.loads(return_val)
        success = False#fixme
        return_val = 'a'#fixme
        if not success:
            pass  # TODO raise exception
        return return_val


connector = None
p = PiMotionSensor(connector, 1, 2)

