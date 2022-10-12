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

    def loop(self):
        self.scheduler.run(blocking=False)

    def cleanup(self):
        self.stop()
        GPIO.cleanup()
        print('LinActuator cleanup done')


def man_t(dir_, t):
    lin_act_pins = {'up': 22, 'down': 4, 'enable': 27}
    lin_act = LinActuator(lin_act_pins)
    lin_act.setup()

    def _exit_code(*args):
        lin_act.cleanup()
        exit(0)

    signal.signal(signal.SIGINT, _exit_code)

    lin_act.drive(dir_, t, blocking=True)
    lin_act.cleanup()


def man_drive():
    from sshkeyboard import listen_keyboard

    lin_act_pins = {'up': 22, 'down': 4, 'enable': 27}
    lin_act = LinActuator(lin_act_pins)
    lin_act.setup()

    def _exit_code(*args):
        lin_act.cleanup()
        exit(0)

    signal.signal(signal.SIGINT, _exit_code)

    def _press(key):
        if key == 'up' or key == 'down':
            lin_act.drive(key, blocking=False)

    def _release(key):
        if key == 'up' or key == 'down':
            lin_act.stop()

    print('Press Esc to quit, or up and down to move actuator..')
    listen_keyboard(
        on_press=_press,
        on_release=_release,
    )

    lin_act.cleanup()


if __name__ == '__main__':
    dir_ = sys.argv[1]
    t = float(sys.argv[2]) if len(sys.argv) > 2 else 4

    if dir_ == 'man':
        man_drive()
    else:
        man_t(dir_, t)
