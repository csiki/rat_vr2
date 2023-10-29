import sys

import numpy
import inspect
import pickle
import socket
import signal
import traceback
from typing import Callable, Any, Union, Tuple, Dict
from copy import deepcopy

from common import *
from motion import MotionSensor, MotionSensors, SmoothMotion
from omni_drive import OmniDrive
from actuator import LinActuator
from player_movement import PlayerMovement, Feedback
from reward import RewardCircuit

import pi_wrapper
from pi_wrapper import PiSmoothMotion, PiMotionSensor, PiOverSocket, PiCallback, PiMotionSensors, PiRewardCircuit


# run loop, parse pc messages over network and call corresponding wrapped functions
# automatically generate wrapper functions from wrapped class declaration

# need to keep track of correspondence between local (real) class objects and server wrapped objects
#   so if a new (wrapped) object is created on server (pc) side and an already existing other (wrapped) object
#   is passed as argument to a function, then here the right (real) object should be passed;
# when having multiple objects of the same type (e.g. MotionSensor), need to keep correspondence (simpler case)
# do this for all function arguments (obj) sent through socket: take type(obj), see if it's in device_clss;
#   if so, find the corresponding real object by obj.wrap_id; as such, keep dict of special objects by their wrap_id

# limitations:
#   no callback possible from pi to pc;
#     as such, both PlayerMovement and Feedback needs to be on pi to OmniDrive.roll() to work

_RUN = True
def _sigint(*args):
    global _RUN
    _RUN = False


def main():

    # TODO argparse
    # remote pc/server address
    server_host = sys.argv[1]
    server_port = int(sys.argv[2]) if len(sys.argv) > 2 else 4444
    do_restart = bool(sys.argv[3]) if len(sys.argv) > 3 else False

    # devices
    device_clss = [MotionSensor, MotionSensors, SmoothMotion, OmniDrive, LinActuator, PlayerMovement, Feedback, RewardCircuit]
    device_cls_names = [cls.__name__ for cls in device_clss]
    device_objs = [dict() for _ in device_clss]  # host id  -> device obj for each class

    device_callback_functions: Dict[int, pi_wrapper.PiCallback] = {}  # host id -> device (wrapped) callback function
    print(f'Listening on {server_host}:{server_port}')

    signal.signal(signal.SIGINT, _sigint)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.connect((server_host, server_port))
        print(f'Connected to server @ {server_host}:{server_port}..')

        while _RUN:

            # receive command
            # cmd example: {'o': id(self), 'c': self.base_cls.__name__, 'f': fun.__name__, 'a': args, 'kwa': kwargs}
            try:
                cmd = sock.recv(4096)
            except ConnectionResetError:
                cmd = ''
            if len(cmd) == 0:
                print('Server disconnected..')
                break

            cmd = pickle.loads(cmd)
            # print('<-', cmd)

            ret_val = None
            exception = None
            callback_ids_to_run = []

            # break free
            if cmd['f'] == '!STOP!':
                print('Remotely stopped..')
                break

            # check command validity
            if cmd['c'] not in device_cls_names:
                exception = ValueError(f'Wrong device class: {cmd["c"]}')
                sock.sendall(pickle.dumps((ret_val, exception, callback_ids_to_run)))  # issue response
                continue

            try:
                device_cls_i = device_cls_names.index(cmd['c'])

                # assign local real objects to device objects passed as arguments
                args = list(cmd['a'])
                kwargs = cmd['kwa']

                def _unwrap_arg(a):
                    if isinstance(a, pi_wrapper.PiOverSocket):
                        return device_objs[device_cls_names.index(type(a).__base__.__name__)][a._wrap_id]
                    elif isinstance(a, pi_wrapper.PiCallback):
                        device_callback_functions[a._wrap_id] = a  # just keep track of callbacks
                        return a
                    return a

                args = [_unwrap_arg(a) for a in args]
                kwargs = {k: _unwrap_arg(a) for k, a in kwargs.items()}

                # create device obj - special case of calling a device function
                if cmd['f'] == '__init__':
                    device_objs[device_cls_i][cmd['o']] = device_clss[device_cls_i](*args, **kwargs)
                    ret_val = id(device_objs[device_cls_i][cmd['o']])

                # any other function call
                else:
                    ret_val = getattr(device_objs[device_cls_i][cmd['o']], cmd['f'])(*args, **kwargs)

                # add callbacks function ids to return if has run
                callback_ids_to_run = [cb._wrap_id for cb in device_callback_functions.values() if cb._called]
                device_callback_functions = {wid: cb for wid, cb in device_callback_functions.items() if not cb._called}

            except Exception as e:
                exception = Exception(str(e) + ' at:\n' + str(traceback.format_exc()))

            # issue response
            # print('->', (ret_val, exception))
            sock.sendall(pickle.dumps((ret_val, exception, callback_ids_to_run)))

    # optional cleanup on each device object
    for device_cls_i, cls in enumerate(device_clss):
        funs = [fname for fname, f in inspect.getmembers(cls, predicate=inspect.isfunction)]
        for obj in device_objs[device_cls_i].values():
            try:
                if 'cleanup' in funs:
                    obj.cleanup()
                    print('cleanup:', obj)
            except Exception as e:
                print('Exception during cleanup:', e)

    return do_restart


if __name__ == '__main__':
    while main(): pass
