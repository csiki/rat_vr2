import sys
import time
import signal
import RPi.GPIO as GPIO


lin_act_pins = {'up': 27, 'down': 4}

GPIO.setmode(GPIO.BCM)

GPIO.setup(lin_act_pins['up'], GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(lin_act_pins['down'], GPIO.OUT, initial=GPIO.LOW)

dir_ = sys.argv[1]
t = int(sys.argv[2]) if len(sys.argv) > 2 else 4


def exit_code(*args):
    GPIO.output(lin_act_pins['up'], GPIO.LOW)
    GPIO.output(lin_act_pins['down'], GPIO.LOW)
    GPIO.cleanup()
    print(' -> done')
    exit(0)


signal.signal(signal.SIGINT, exit_code)

print(f'{dir_} for {t} sec')
GPIO.output(lin_act_pins['up' if dir_ == 'down' else 'down'], GPIO.LOW)
GPIO.output(lin_act_pins[dir_], GPIO.HIGH)
time.sleep(t)

exit_code()
