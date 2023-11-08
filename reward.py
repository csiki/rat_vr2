import sys
import time

import serial
from functools import reduce
import threading
from config import *

# Data format: $DOOM,[Valve Open Millisec(int)],[Pressure SetPoint(float)],
#   [Pump Override Control Millisec(int)],[Left Blow Millisec(int)],
#   [Right Blow Millisec(int)],[Stepper Turns(int)]*CheckSum8Xor
# e.g.: $DOOM,5000,94.4,5000,2500,3500,3*2C


class RewardCircuit:
    RESP_BEG_STR = ' > > > '
    RESP_END_STR = 'kPa'
    BAUDRATE = 57600
    WAIT_TIME = .06  # sec

    def __init__(self, serial_port, init_pressure_setpoint=PRESSURE_SETPOINT, valve_ms_per_ul=VALVE_MS_PER_UL,
                 run_on_sep_thread=True, auto_mixing_at_every=None, run_stat_in_every=0.2):
        self.pressure_setpoint = init_pressure_setpoint
        self.valve_ms_per_ul = valve_ms_per_ul
        # degree of bump to left/right where the opposite puffer is at 0
        # there is a gradient of puffing power of the opposite puffer from 0 to the sides at this degree
        # this is true too for bump angles at the +-90 +- delta ranges
        # TODO for now, puff strength cannot be set, only duration,
        #   used for left/right differential puffing

        self.auto_mixing_at_every = auto_mixing_at_every  # at every x second it turns 5 times
        self.last_mixing = time.time()
        self.rc_state = None  # last polled state
        self.rc_state_mut = []
        self.do_stop = False
        self.pressure_setpoint_changed = False

        self.run_stat_in_every = run_stat_in_every
        self.stat_last_run = 0

        # buffer variables
        self._reset(init_pressure_setpoint)

        # threading
        self.run_on_sep_thread = run_on_sep_thread
        self.thread: threading.Thread = None

        self.ser = serial.Serial(serial_port, baudrate=RewardCircuit.BAUDRATE, timeout=RewardCircuit.WAIT_TIME)
        time.sleep(4)  # wait until arduino sets up
        self.update(pressure_setpoint=init_pressure_setpoint)

    def update(self, valve_open_ms=0, pressure_setpoint=None, pump_override_ctrl=0,
               left_blow_ms=0, right_blow_ms=0, press_lever_ms=0, mixer_turns=0):
        if pressure_setpoint is not None:
            self.pressure_setpoint_changed = True
        self.valve_open_ms = self.valve_open_ms + valve_open_ms
        self.pressure_setpoint = pressure_setpoint if pressure_setpoint is not None else self.pressure_setpoint
        self.pump_override_ctrl = pump_override_ctrl
        self.left_blow_ms = max(self.left_blow_ms, left_blow_ms)
        self.right_blow_ms = max(self.right_blow_ms, right_blow_ms)
        self.press_lever_ms = max(self.press_lever_ms, press_lever_ms)
        self.mixer_turns = max(self.mixer_turns, mixer_turns)

    def loop(self, verbose=False):
        if self.auto_mixing_at_every and self.mixer_turns == 0 \
                and time.time() - self.last_mixing > self.auto_mixing_at_every:
            self.last_mixing = time.time()
            self.mixer_turns = 5

        nothing_todo = (self.valve_open_ms + self.left_blow_ms + self.right_blow_ms + self.pump_override_ctrl +
                        self.press_lever_ms + self.mixer_turns) == 0 and not self.pressure_setpoint_changed

        proc = None
        if self.do_stop:
            proc = lambda: self._stop()
        elif not nothing_todo:  # there's something to send
            proc = lambda: self._update(self.valve_open_ms, self.pressure_setpoint, self.pump_override_ctrl,
                                        self.left_blow_ms, self.right_blow_ms, self.press_lever_ms,
                                        self.mixer_turns, verbose, self.rc_state_mut)
        elif nothing_todo and time.time() - self.stat_last_run > self.run_stat_in_every:
            proc = lambda: self._stat(verbose, self.rc_state_mut)

        if len(self.rc_state_mut) > 0:
            self.rc_state = self.rc_state_mut[-1]
            self.rc_state_mut.clear()

        if proc:
            self.stat_last_run = time.time()
        else:  # nothing to do
            return

        if self.thread is not None and self.thread.is_alive():
            self.thread.join()

        if self.run_on_sep_thread:
            self.thread = threading.Thread(target=proc)
            self.thread.start()
        else:
            proc()

        self._reset(self.pressure_setpoint)
        return self.rc_state

    def _stat(self, verbose=False, rc_state_out: list = None):
        cmd = f'STAT'
        xor_sum = reduce(lambda a, b: a ^ b, map(ord, cmd), 0)
        cmd = f'${cmd}*{hex(xor_sum)[2:].upper()}'
        self.ser.write(str.encode(cmd))
        time.sleep(RewardCircuit.WAIT_TIME)

        resp = self.ser.readline().decode()
        rc_state = self._proc_resp(resp, cmd, verbose)
        rc_state_out.append(rc_state)

    def _update(self, valve_open_ms=0, pressure_setpoint=None, pump_override_ctrl=0,
                left_blow_ms=0, right_blow_ms=0, press_lever_ms=0, mixer_turns=0, verbose=False,
                rc_state_out: list = None):

        # sticky pressure setpoint
        self.pressure_setpoint = self.pressure_setpoint if pressure_setpoint is None else pressure_setpoint
        cmd = f'DOOM,{valve_open_ms:.0f},{self.pressure_setpoint:.2f},{pump_override_ctrl:.0f},' \
              f'{left_blow_ms:.0f},{right_blow_ms:.0f},{press_lever_ms:.0f},{mixer_turns:.0f}'

        xor_sum = reduce(lambda a, b: a ^ b, map(ord, cmd), 0)
        cmd = f'${cmd}*{hex(xor_sum)[2:].upper()}'
        self.ser.write(str.encode(cmd))
        time.sleep(RewardCircuit.WAIT_TIME)

        resp = self.ser.readline().decode()
        rc_state = self._proc_resp(resp, cmd, verbose)
        rc_state_out.append(rc_state)

    def _proc_resp(self, resp, cmd, verbose=False):
        if verbose:
            print('resp:', resp)
        # resp = resp[len(cmd):]  # rm beg/echo

        rc_state = None
        try:
            # if True:
            resp = resp[resp.index(RewardCircuit.RESP_BEG_STR) + len(RewardCircuit.RESP_BEG_STR):
                        resp.index(RewardCircuit.RESP_END_STR)].replace(' ', '')
            rc_state = [s.split(':') for s in resp.split('|')]
            rc_state = {name: float(val) for name, val in rc_state}
            # self.rc_state = rc_state
            if verbose:
                print('cmd:', cmd)
                print('Reward response:', rc_state)

        except ValueError as e:
            if verbose:
                print('INVALID RESPONSE:', resp, file=sys.stderr)
                print('Exception:', e)

        return rc_state

    def _reset(self, init_pressure_setpoint=PRESSURE_SETPOINT):
        self.valve_open_ms = 0
        self.pressure_setpoint = init_pressure_setpoint
        self.pump_override_ctrl = 0
        self.left_blow_ms = 0
        self.right_blow_ms = 0
        self.press_lever_ms = 0
        self.mixer_turns = 0
        self.do_stop = False
        self.pressure_setpoint_changed = False

    def open_valve(self, ms: int = None, ul: int = None):
        assert (ms is None) ^ (ul is None)
        ms = ul * self.valve_ms_per_ul if ul is not None else ms
        self.update(valve_open_ms=ms)

    @staticmethod
    def calc_puff_from_wall_bump(b_angle, b_distance, puff_delta_share_degree=10, puff_within_distance=1,
                                 puff_base_dur=100, return_cmd=False):
        # puff rules: at 0 degree both left and right blows at max=1
        #   from -puff_delta_share_degree to 0, a gradient of blow from right from 0 to 1
        #   from puff_delta_share_degree to 0, a gradient of blow from left from 0 to 1
        #   at <=0 left blows \1, at >=0 right blows 1, otherwise 0 blow
        puff_d = puff_delta_share_degree
        left_puff = float(-90 <= b_angle <= 0) + (0 < b_angle < puff_d) * (1 - (b_angle / puff_d)) \
            + (-90 - puff_d < b_angle < -90) * (1 - (-b_angle - 90) / puff_d)
        right_puff = float(0 <= b_angle <= 90) + (-puff_d <= b_angle < 0) * (1 - (b_angle / -puff_d)) \
            + (90 < b_angle < 90 + puff_d) * (1 - (b_angle - 90) / puff_d)

        # linear puff strength from 0 to 1, by puff_within_distance distance to 0
        puff_dist_scaler = max(0, 1 - b_distance / puff_within_distance)

        if return_cmd:
            return dict(left_blow_ms=puff_base_dur * left_puff * puff_dist_scaler,
                        right_blow_ms=puff_base_dur * right_puff * puff_dist_scaler)
        return left_puff * puff_dist_scaler, right_puff * puff_dist_scaler

    def stop(self):
        self.do_stop = True

    def _stop(self):
        cmd = 'STOP,,,,,'
        xor_sum = reduce(lambda a, b: a ^ b, map(ord, cmd), 0)
        cmd = f'${cmd}*{hex(xor_sum)[2:].upper()}'  # $STOP,,,,,*34
        self.ser.write(str.encode(cmd))

    def cleanup(self):
        self.stop()
        self.loop()
        time.sleep(0.5)
        if self.thread is not None and self.thread.is_alive():
            self.thread.join()
        self.ser.close()
