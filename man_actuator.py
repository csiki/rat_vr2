import sys
import time
import signal
import RPi.GPIO as GPIO


lin_act_pins = {'up': 22, 'down': 4, 'enable': 27}

GPIO.setmode(GPIO.BCM)

GPIO.setup(lin_act_pins['up'], GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(lin_act_pins['down'], GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(lin_act_pins['enable'], GPIO.OUT, initial=GPIO.LOW)

dir_ = sys.argv[1]
t = int(sys.argv[2]) if len(sys.argv) > 2 else 4


def exit_code(*args):
    GPIO.output(lin_act_pins['enable'], GPIO.LOW)
    GPIO.output(lin_act_pins['up'], GPIO.LOW)
    GPIO.output(lin_act_pins['down'], GPIO.LOW)
    GPIO.cleanup()
    print(' -> done')
    exit(0)


signal.signal(signal.SIGINT, exit_code)

print(f'{dir_} for {t} sec')
print('  low:', lin_act_pins['up' if dir_ == 'down' else 'down'])
GPIO.output(lin_act_pins['up' if dir_ == 'down' else 'down'], GPIO.LOW)
print('  high:', lin_act_pins[dir_])
GPIO.output(lin_act_pins[dir_], GPIO.HIGH)
print('  enable:', lin_act_pins['enable'])
GPIO.output(lin_act_pins['enable'], GPIO.HIGH)

time.sleep(t)

exit_code()
