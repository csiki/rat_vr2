import serial
from functools import reduce

# Data format: $DOOM,[Valve Open Millisec(int)],[Pressure SetPoint(float)],
#   [Pump Override Control Millisec(int)],[Left Blow Millisec(int)],
#   [Right Blow Millisec(int)],[Stepper Turns(int)]*CheckSum8Xor
# e.g.: $DOOM,5000,94.4,5000,2500,3500,3*2C


class RewardCircuit:
    def __init__(self, serial_port, pressure_setpoint):
        self.pressure_setpoint = pressure_setpoint
        self.ser = serial.Serial(serial_port, baudrate=57600, timeout=0.05)

    def send_cmd(self, valve_open_ms=0, pressure_setpoint=None, pump_override_ctrl=0,
                 left_blow_ms=0, right_blow_ms=0, mixer_turns=0):
        # TODO separate commands
        # TODO have mixer_turns as on-off instead of number of turns
        pressure_setpoint = self.pressure_setpoint if pressure_setpoint is None else pressure_setpoint
        cmd = f'DOOM,{valve_open_ms:.0f},{pressure_setpoint:.2f},{pump_override_ctrl:.0f},' \
              f'{left_blow_ms:.0f},{right_blow_ms:.0f},{mixer_turns:.0f}'

        xor_sum = reduce(lambda a, b: a ^ b, map(ord, cmd), 0)
        cmd = f'${cmd}*{hex(xor_sum)[2:].upper()}'
        self.ser.write(str.encode(cmd))

        resp = self.ser.readline().decode()
        print('Reward response:', resp)  # TODO

    def cleanup(self):
        self.ser.close()
