import numpy
import pickle
import socket
from motion import MotionSensor, MotionSensors, SmoothMotion


# TODO run loop, parse pc messages over network and call corresponding wrapped functions
# TODO automatically generate wrapper functions from wrapped class declaration

# devices
device_clss = [MotionSensor, MotionSensors, SmoothMotion]

# TODO need to keep track of correspondence between local (real) class objects and server wrapped objects
#   so if a new (wrapped) object is created on server side and an already existing other (wrapped) object
#   is passed as argument to init, then here the right (real) object should be passed;
#   also, when having multiple objects of the same type (e.g. MotionSensor), need to keep correspondence (simpler case)
# TODO do this for all function arguments (obj) sent through socket: take type(obj), see if it's in device_clss;
#   if so, find the corresponding real object by obj.host_id; as such, keep dict of special objects by their host_id


server_name = 'LAPTOP-AOH10NOP'
server_host = socket.gethostbyname(server_name)  # '192.168.0.129'
server_port = 4444

# TODO test:
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((server_host, server_port))
    data = s.recv(2048)
    data = pickle.loads(data)
    print(f"Received: {data}")
    s.sendall(pickle.dumps(('return_value', None)))
