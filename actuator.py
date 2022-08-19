import sys
import time
import signal
import sched
import RPi.GPIO as GPIO


class LinActuator:
    def __init__(self, pins):
        self.pins = pins
        self.scheduler = sched.scheduler(time.time, time.sleep)
        GPIO.setmode(GPIO.BCM)

    def setup(self):
        GPIO.setup(self.pins['up'], GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pins['down'], GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pins['enable'], GPIO.OUT, initial=GPIO.LOW)
        print('LinActuator setup done')

    def drive(self, dir_, t=None, blocking=False):
        opp_dir = 'up' if dir_ == 'down' else 'down'
        GPIO.output(self.pins[opp_dir], GPIO.LOW)
        GPIO.output(self.pins[dir_], GPIO.HIGH)
        GPIO.output(self.pins['enable'], GPIO.HIGH)

        if t is not None and blocking:
            time.sleep(t)
            self.stop()
        elif t is not None and not blocking:
            self.scheduler.enter(t, 1, self.stop)

    def stop(self):
        GPIO.output(self.pins['enable'], GPIO.LOW)
        GPIO.output(self.pins['up'], GPIO.LOW)
        GPIO.output(self.pins['down'], GPIO.LOW)

    def cleanup(self):
        self.stop()
        GPIO.cleanup()
        print('LinActuator cleanup done')


def man(dir_, t):
    lin_act_pins = {'up': 22, 'down': 4, 'enable': 27}
    lin_act = LinActuator(lin_act_pins)
    lin_act.setup()

    def exit_code(*args):
        lin_act.cleanup()
        exit(0)

    signal.signal(signal.SIGINT, exit_code)

    lin_act.drive(dir_, t, blocking=True)
    lin_act.cleanup()


if __name__ == '__main__':
    dir_ = sys.argv[1]
    t = int(sys.argv[2]) if len(sys.argv) > 2 else 4

    man(dir_, t)
