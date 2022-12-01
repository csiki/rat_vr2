import serial
from functools import reduce
from config import *

# Data format: $DOOM,[Valve Open Millisec(int)],[Pressure SetPoint(float)],
#   [Pump Override Control Millisec(int)],[Left Blow Millisec(int)],
#   [Right Blow Millisec(int)],[Stepper Turns(int)]*CheckSum8Xor
# e.g.: $DOOM,5000,94.4,5000,2500,3500,3*2C


class RewardCircuit:
    RESP_BEG_STR = ' > > > '
    RESP_END_STR = 'kPa'

    def __init__(self, serial_port, pressure_setpoint=PRESSURE_SETPOINT, valve_ms_per_ul=VALVE_MS_PER_UL):
        self.pressure_setpoint = pressure_setpoint
        self.valve_ms_per_ul = valve_ms_per_ul
        self.ser = serial.Serial(serial_port, baudrate=57600, timeout=0.05)

    def send_cmd(self, valve_open_ms=0, pressure_setpoint=None, pump_override_ctrl=0,
                 left_blow_ms=0, right_blow_ms=0, mixer_turns=0):
        # TODO separate commands
        # TODO have mixer_turns and right/left blows as on-off number of turns or ms to blow
        pressure_setpoint = self.pressure_setpoint if pressure_setpoint is None else pressure_setpoint
        cmd = f'DOOM,{valve_open_ms:.0f},{pressure_setpoint:.2f},{pump_override_ctrl:.0f},' \
              f'{left_blow_ms:.0f},{right_blow_ms:.0f},{mixer_turns:.0f}'

        xor_sum = reduce(lambda a, b: a ^ b, map(ord, cmd), 0)
        cmd = f'${cmd}*{hex(xor_sum)[2:].upper()}'
        self.ser.write(str.encode(cmd))

        resp = self.ser.readline().decode()
        resp = resp[resp.index(RewardCircuit.RESP_BEG_STR) + len(RewardCircuit.RESP_BEG_STR):
                    resp.index(RewardCircuit.RESP_END_STR)].replace(' ', '')
        rc_state = [s.split(':') for s in resp.split('|')]
        rc_state = {name: float(val) for name, val in rc_state}

        print('Reward response:', rc_state)  # TODO

    def open_valve(self, ms: int = None, ul: int = None):
        assert (ms is None) ^ (ul is None)
        ms = ul * self.valve_ms_per_ul if ul is not None else ms
        self.send_cmd(valve_open_ms=ms)

    def stop(self):
        cmd = 'STOP,,,,,'
        xor_sum = reduce(lambda a, b: a ^ b, map(ord, cmd), 0)
        cmd = f'${cmd}*{hex(xor_sum)[2:].upper()}'
        self.ser.write(str.encode(cmd))

    def cleanup(self):
        self.ser.close()
