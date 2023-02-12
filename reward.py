import time

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

    def __init__(self, serial_port, pressure_setpoint=PRESSURE_SETPOINT, valve_ms_per_ul=VALVE_MS_PER_UL,
                 puff_delta_share_degree=10, puff_within_distance=1):
        self.pressure_setpoint = pressure_setpoint
        self.valve_ms_per_ul = valve_ms_per_ul
        # degree of bump to left/right where the opposite puffer is at 0
        # there is a gradient of puffing power of the opposite puffer from 0 to the sides at this degree
        # this is true too for bump angles at the +-90 +- delta ranges
        self.puff_delta_share_degree = puff_delta_share_degree
        self.puff_within_distance = puff_within_distance

        self.ser = serial.Serial(serial_port, baudrate=57600, timeout=0.05)
        time.sleep(4)  # wait until arduino sets up

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

        time.sleep(0.1)  # TODO remove prints, don't read shit, this is way too slow

        resp = self.ser.readline().decode()
        resp = resp[len(cmd):]  # rm beg
        print('resp:', resp)

        resp = resp[resp.index(RewardCircuit.RESP_BEG_STR) + len(RewardCircuit.RESP_BEG_STR):
                    resp.index(RewardCircuit.RESP_END_STR)].replace(' ', '')
        rc_state = [s.split(':') for s in resp.split('|')]
        rc_state = {name: float(val) for name, val in rc_state}

        print('Reward response:', rc_state)  # TODO

    def open_valve(self, ms: int = None, ul: int = None):
        assert (ms is None) ^ (ul is None)
        ms = ul * self.valve_ms_per_ul if ul is not None else ms
        self.send_cmd(valve_open_ms=ms)

    def calc_puff_from_wall_bump(self, b_angle, b_distance):
        # puff rules: at 0 degree both left and right blows at max=1
        #   from -puff_delta_share_degree to 0, a gradient of blow from right from 0 to 1
        #   from puff_delta_share_degree to 0, a gradient of blow from left from 0 to 1
        #   at <=0 left blows \1, at >=0 right blows 1, otherwise 0 blow
        puff_d = self.puff_delta_share_degree
        left_puff = float(-90 <= b_angle <= 0) + (0 < b_angle < puff_d) * (1 - (b_angle / puff_d)) \
            + (-90 - puff_d < b_angle < -90) * (1 - (-b_angle - 90) / puff_d)
        right_puff = float(0 <= b_angle <= 90) + (-puff_d <= b_angle < 0) * (1 - (b_angle / -puff_d)) \
            + (90 < b_angle < 90 + puff_d) * (1 - (b_angle - 90) / puff_d)

        # linear puff strength from 0 to 1, by puff_within_distance distance to 0
        puff_dist_scaler = max(0, 1 - b_distance / self.puff_within_distance)

        return left_puff * puff_dist_scaler, right_puff * puff_dist_scaler

    def stop(self):
        cmd = 'STOP,,,,,'
        xor_sum = reduce(lambda a, b: a ^ b, map(ord, cmd), 0)
        cmd = f'${cmd}*{hex(xor_sum)[2:].upper()}'  # $STOP,,,,,*34
        self.ser.write(str.encode(cmd))

    def cleanup(self):
        self.ser.close()
