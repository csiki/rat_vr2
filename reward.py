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


# TODO redo this, so it runs not in separate thread, but once in loop


class RewardCircuit:
    RESP_BEG_STR = ' > > > '
    RESP_END_STR = 'kPa'

    def __init__(self, serial_port, init_pressure_setpoint=PRESSURE_SETPOINT, valve_ms_per_ul=VALVE_MS_PER_UL,
                 puff_delta_share_degree=10, puff_within_distance=1, puff_base_dur=100, auto_mixing_at_every=None):
        self.pressure_setpoint = init_pressure_setpoint
        self.valve_ms_per_ul = valve_ms_per_ul
        # degree of bump to left/right where the opposite puffer is at 0
        # there is a gradient of puffing power of the opposite puffer from 0 to the sides at this degree
        # this is true too for bump angles at the +-90 +- delta ranges
        self.puff_delta_share_degree = puff_delta_share_degree
        self.puff_within_distance = puff_within_distance
        self.puff_base_dur = puff_base_dur
        # TODO for now, puff strength cannot be set, only duration,
        #   used for left/right differential puffing

        self.auto_mixing_at_every = auto_mixing_at_every  # at every x second it turns 5 times
        self.last_mixing = time.time()
        self.rc_state = None  # last polled state
        self.has_updated_state = False

        # threading
        self.cmd_thread = None
        self.is_running = False
        self.cmd_lock = threading.Lock()

        self.ser = serial.Serial(serial_port, baudrate=57600, timeout=0.05)
        time.sleep(4)  # wait until arduino sets up
        self.send(pressure_setpoint=init_pressure_setpoint)

    def stat(self, verbose=False):
        cmd = f'STAT'
        xor_sum = reduce(lambda a, b: a ^ b, map(ord, cmd), 0)
        cmd = f'${cmd}*{hex(xor_sum)[2:].upper()}'

        if self._can_run():
            self.ser.write(str.encode(cmd))
            time.sleep(0.02)  # TODO too slow
            resp = self.ser.readline().decode()
            with self.cmd_lock:
                self.is_running = False
            self._proc_resp(resp, cmd, verbose)

        return self.rc_state

    def send(self, valve_open_ms=0, pressure_setpoint=None, pump_override_ctrl=0,
             left_blow_ms=0, right_blow_ms=0, press_lever_ms=0, mixer_turns=0, verbose=False):

        if self.auto_mixing_at_every and mixer_turns == 0 and \
                time.time() - self.last_mixing > self.auto_mixing_at_every:
            self.last_mixing = time.time()
            mixer_turns = 5

        kwargs = locals()
        kwargs.pop('self')

        nothing_todo = valve_open_ms + left_blow_ms + right_blow_ms + pump_override_ctrl + press_lever_ms + mixer_turns == 0 \
                       and pressure_setpoint is None
        if nothing_todo:
            return self.rc_state

        if self._can_run():
            self.cmd_thread = threading.Thread(target=self._send, kwargs=kwargs)
            self.cmd_thread.start()
        # else:  TODO add commands to queue, not to miss them, use concurrent.futures.ThreadPoolExecutor

        return self.rc_state

    def _send(self, valve_open_ms=0, pressure_setpoint=None, pump_override_ctrl=0,
                 left_blow_ms=0, right_blow_ms=0, press_lever_ms=0, mixer_turns=0, verbose=False):
        # TODO separate commands
        # TODO have mixer_turns and right/left blows as on-off number of turns or ms to blow
        # TODO run on separate thread

        # sticky pressure setpoint
        self.pressure_setpoint = self.pressure_setpoint if pressure_setpoint is None else pressure_setpoint
        cmd = f'DOOM,{valve_open_ms:.0f},{self.pressure_setpoint:.2f},{pump_override_ctrl:.0f},' \
              f'{left_blow_ms:.0f},{right_blow_ms:.0f},{press_lever_ms:.0f},{mixer_turns:.0f}'

        xor_sum = reduce(lambda a, b: a ^ b, map(ord, cmd), 0)
        cmd = f'${cmd}*{hex(xor_sum)[2:].upper()}'
        self.ser.write(str.encode(cmd))

        time.sleep(0.02)  # TODO remove prints, don't read shit, this is way too slow

        resp = self.ser.readline().decode()
        self._proc_resp(resp, cmd, verbose)

        with self.cmd_lock:
            self.is_running = False
        return self.rc_state

    def _can_run(self):
        run_it = False
        with self.cmd_lock:
            if not self.is_running:
                if self.cmd_thread:
                    self.cmd_thread.join()
                self.is_running = True
                run_it = True
            # else:  TODO add commands to queue, not to miss them, use concurrent.futures.ThreadPoolExecutor
        return run_it

    def _proc_resp(self, resp, cmd, verbose=False):
        if verbose:
            print('resp:', resp)
        resp = resp[len(cmd):]  # rm beg/echo

        try:
            resp = resp[resp.index(RewardCircuit.RESP_BEG_STR) + len(RewardCircuit.RESP_BEG_STR):
                        resp.index(RewardCircuit.RESP_END_STR)].replace(' ', '')
            rc_state = [s.split(':') for s in resp.split('|')]
            self.rc_state = {name: float(val) for name, val in rc_state}
            self.has_updated_state = True
            # TODO remove response, or have it as an option; we will probably not need it:
            #   or move reading it to a separate thread
            if verbose:
                print('cmd:', cmd)
                print('Reward response:', rc_state)  # TODO

        except ValueError as e:
            if verbose:
                print('INVALID RESPONSE:', resp, file=sys.stderr)
                print('Exception:', e)

    def update_if_havent(self):
        if not self.has_updated_state:
            self.stat()

    def reset_state_update_checker(self):
        self.has_updated_state = False

    def open_valve(self, ms: int = None, ul: int = None):
        assert (ms is None) ^ (ul is None)
        ms = ul * self.valve_ms_per_ul if ul is not None else ms
        self.send(valve_open_ms=ms)

    def calc_puff_from_wall_bump(self, b_angle, b_distance, return_cmd=False):
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

        if return_cmd:
            return dict(left_blow_ms=self.puff_base_dur * left_puff * puff_dist_scaler,
                        right_blow_ms=self.puff_base_dur * right_puff * puff_dist_scaler)
        return left_puff * puff_dist_scaler, right_puff * puff_dist_scaler

    def stop(self):
        cmd = 'STOP,,,,,'
        xor_sum = reduce(lambda a, b: a ^ b, map(ord, cmd), 0)
        cmd = f'${cmd}*{hex(xor_sum)[2:].upper()}'  # $STOP,,,,,*34
        self.ser.write(str.encode(cmd))

    def cleanup(self):
        self.stop()
        time.sleep(0.5)
        self.ser.close()
