import time
from pi_wrapper import PiSmoothMotion, PiMotionSensor, PiMotionSensors, PiOmniDrive, PiFeedback, \
    PiPlayerMovement, ServerSocket, PiRewardCircuit


# pc/server address
host, port = '192.168.1.74', 4444  # TODO as cmd argument
serial_port = '/dev/ttyACM0'

with ServerSocket(host, port) as conn:
    reward_circuit = PiRewardCircuit(conn, serial_port)

    for i in range(100):
        print('pump', i)
        s_ = time.time()
        reward_circuit.update(valve_open_ms=5000)
        while time.time() - s_ < 5.:
            reward_circuit.loop()
            time.sleep(0.001)

