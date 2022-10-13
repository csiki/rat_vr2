import numpy
import inspect
import pickle
import socket

from motion import MotionSensor, MotionSensors, SmoothMotion
from omni_drive import OmniDrive
from actuator import LinActuator
from player_movement import PlayerMovement, Feedback
import pi_wrapper


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

def main():

    # devices  # TODO add lever, reward, trainer
    device_clss = [MotionSensor, MotionSensors, SmoothMotion, OmniDrive, LinActuator, PlayerMovement, Feedback]
    device_cls_names = [cls.__name__ for cls in device_clss]
    device_objs = [dict() for _ in device_clss]  # host id  -> device obj for each class

    server_name = 'LAPTOP-AOH10NOP'
    server_host = socket.gethostbyname(server_name)  # '192.168.0.129'
    server_port = 4444

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        # TODO test
        sock.connect((server_host, server_port))
        data = sock.recv(4096)
        data = pickle.loads(data)
        print(f"Received: {data}")
        sock.sendall(pickle.dumps(('return_value', None)))
        # TODO test end

        while True:  # TODO test
            # cmd example: {'o': id(self), 'c': self.base_cls.__name__, 'f': fun.__name__, 'a': args, 'kwa': kwargs}
            cmd = pickle.loads(sock.recv(4096))
            ret_val = None
            exception = None

            # break free
            if cmd['f'] == '!STOP!':
                break

            # check command validity
            if cmd['c'] not in device_cls_names:
                exception = ValueError(f'Wrong device class: {cmd["c"]}')
                sock.sendall(pickle.dumps((ret_val, exception)))  # issue response
                continue

            try:
                device_cls_i = device_cls_names.index(cmd['c'])

                # assign local real objects to device objects passed as arguments
                args = list(cmd['a'])
                kwargs = cmd['kwa']
                dev_from_wrap_obj = lambda o: device_objs[device_cls_names.index(type(o).__base__.__name__)][o.wrap_id]

                args = [dev_from_wrap_obj(a) if isinstance(a, pi_wrapper.PiOverSocket) else a
                        for a in args]
                kwargs = {k: dev_from_wrap_obj(a) if isinstance(a, pi_wrapper.PiOverSocket) else a
                          for k, a in kwargs.items()}

                # create device obj - special case of calling a device function
                if cmd['f'] == '__init__':
                    device_objs[device_cls_i][cmd['o']] = device_clss[device_cls_i](*args, **kwargs)
                    ret_val = id(device_objs[device_cls_i][cmd['o']])

                # any other function call
                else:
                    ret_val = getattr(device_objs[device_cls_i][cmd['o']], cmd['f'])(*args, **kwargs)

            except Exception as e:
                exception = e

            # issue response
            sock.sendall(pickle.dumps((ret_val, exception)))

    # optional cleanup on each device object
    for device_cls_i, cls in enumerate(device_clss):
        for obj in device_objs[device_cls_i]:
            if 'cleanup' in inspect.getmembers(cls, predicate=inspect.isfunction):
                obj.cleanup()


if __name__ == '__main__':
    main()
