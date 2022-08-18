# simple "turn and move straight" dynamics and motion planning
#   A) to construct the motion map, manually run through the game map a couple times,
#      record paths, take points from the paths, divide map into squares,
#      cluster points inside squares into single mean point,
#      register paths between touching squares (diagonals too), and take mean path between;
#      have an optimal run, register squares passed and that should be the final motion plan
#   B) hand-design a graph on the map with positions and connections between, then define sequence of points as motion
#   => once optimal motion with sequence of steps is available,
#      every step taken negates the necessity to do the prev steps
#   -- enable backward movement in some cases: typically when there is an overshoot of movement
#      that can be fixed with a small (< X) backward movement, where the correct angle of movement is <20 degrees
#      off from the backward direction => make such correction movements smooth turning and movement at the same time

# trapezoidal velocity change to constant, predefined velocity
#   wheel acceleration/deacc should be the same as the acceleration/deacc in Doom to simplify speed transfer
# feedforward + PI feedback control: first turn then move

# if there is a height differential between roller vs bearing ball mount, which should be minimized 'cause screen..
#   place motion sensors (back and left side) at the different heights;
#   forward/backward readings will work no matter what on the back sensor,
#   strafe roll readings will work on the left sensor,
#   take turn readings from the sensor at height corresponding to the roller or bearing mount to be more accurate

# according to:
#   https://www.researchgate.net/publication/256089847_Dynamical_Models_for_Omni-directional_Robots_with_3_and_4_Wheels
# ball position: x, y, theta -> irrelevant
# game position: xg, yg, theta_g -> given
#   xg = Gx * x; yg = Gy * y; theta ~= theta_g

# game linear motion vector -> delta movement of ball -> ball velocity planning
#   -> wheel speeds calc -> START -> read motion sensor -> game movement -> read game position
#   -> calc motion error -...-> adjust wheel speeds

# calibrate w/o a rat on:
#   1) fit direction: on each axis the planned direction of constant motion should be the same as the sensed motion
#      => params: elements of the matrix in eq 2 of the paper above
#      => how: input is vt, vnt, wt; output is v0, v1, v2; params in the mx;
#   2) fit full turn: draw a vertical line on ball, turn the ball around until the line shows again
#                     auto or manual detection of line; if possible, use the sensor's absolute pos measurement
#      => derive: (sensed) motion / degree
#   3) fit acc/deacc: make the ball move the same amount as the character in-game times a predefined constant
#      => params: PI controller constants, acceleration/deacc time

import time
import numpy as np
import RPi.GPIO as GPIO

roller_dirs = np.array(['left', 'right'])
d = (10. + 6.75) / 100.  # m, wheel distance
trans_mx = np.array(
    [[-np.sin(np.pi / 3), np.cos(np.pi / 3), d],
     [0, -1, d],
     [np.sin(np.pi / 3), np.cos(np.pi / 3), d]]
)

lin_act_pins = {'up': 27, 'down': 4}
roller2_pins = {'right': 25, 'left': 26, 'pwm': 12}
roller1_pins = {'right': 5, 'left': 6, 'pwm': 13}
roller0_pins = {'right': 23, 'left': 24, 'pwm': 18}
roller_pins = [roller0_pins, roller1_pins, roller2_pins]

pin_sets = [lin_act_pins, *roller_pins]

GPIO.setmode(GPIO.BCM)

# setup
pwms = []
pwm_freq = 1000
for pins in pin_sets:
    for n in pins.values():
        GPIO.setup(n, GPIO.OUT, initial=GPIO.LOW)

    # setup pwms
    if 'pwm' in pins:
        pwm = GPIO.PWM(pins['pwm'], pwm_freq)
        pwms.append(pwm)

# straight test
print('straight test')
drive_v = np.array([1., 0., 0.])  # v, vn, w

def calc_wheel_v(drive_v):
    wheel_v = np.matmul(trans_mx, drive_v.transpose())
    wheel_v = wheel_v / np.abs(wheel_v).max()

    wheel_dc = np.abs(wheel_v * 100.)  # duty cycle max 100
    wheel_dir = roller_dirs[(wheel_v > 0).astype(int)]

    return wheel_dir, wheel_dc

wheel_dir, wheel_dc = calc_wheel_v(drive_v)

def drive(wheel_dir, wheel_dc):
    # set duty cycle
    for i, pins in enumerate(roller_pins):
        GPIO.output(pins['left'], GPIO.LOW)
        GPIO.output(pins['right'], GPIO.LOW)
        pwms[i].start(wheel_dc[i])

    # start
    for i, pins in enumerate(roller_pins):
        GPIO.output(pins[wheel_dir[i]], GPIO.HIGH)

def stop_wheels():
    for i, pins in enumerate(roller_pins):
        GPIO.output(pins['left'], GPIO.LOW)
        GPIO.output(pins['right'], GPIO.LOW)
        pwms[i].stop()

drive(wheel_dir, wheel_dc)
time.sleep(5)
stop_wheels()
time.sleep(2)

# right strafe test
print('right strafe test')
drive_v = np.array([0., -1., 0.])  # v, vn, w  # TODO -1 ?

wheel_dir, wheel_dc = calc_wheel_v(drive_v)
drive(wheel_dir, wheel_dc)
time.sleep(5)
stop_wheels()
time.sleep(2)

# left strafe test
print('right strafe test')
drive_v = np.array([0., 1., 0.])  # v, vn, w

wheel_dir, wheel_dc = calc_wheel_v(drive_v)
drive(wheel_dir, wheel_dc)
time.sleep(5)
stop_wheels()
time.sleep(2)

# counter-clockwise turn test
print('counter-clockwise turn test')
drive_v = np.array([0., 0., 1.])  # v, vn, w

wheel_dir, wheel_dc = calc_wheel_v(drive_v)
drive(wheel_dir, wheel_dc)
time.sleep(5)
stop_wheels()
time.sleep(2)

# clockwise turn test
print('counter-clockwise turn test')
drive_v = np.array([0., 0., -1.])  # v, vn, w

wheel_dir, wheel_dc = calc_wheel_v(drive_v)
drive(wheel_dir, wheel_dc)
time.sleep(5)
stop_wheels()
time.sleep(2)

# clockwise turn test
print('counter-clockwise turn test')
drive_v = np.array([0., 0., -1.])  # v, vn, w

wheel_dir, wheel_dc = calc_wheel_v(drive_v)
drive(wheel_dir, wheel_dc)
time.sleep(5)
stop_wheels()
time.sleep(2)

# done
GPIO.cleanup()

# TODO try hardware pwm in C?
#   https://www.electronicwings.com/raspberry-pi/raspberry-pi-pwm-generation-using-python-and-c

# TODO class OmniDrive:

