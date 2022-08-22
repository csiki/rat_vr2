import time
import argparse
import numpy as np
from pmw3901 import PMW3901, PAA5100, BG_CS_FRONT_BCM, BG_CS_BACK_BCM


class MotionSensor:
    def __init__(self, spi_port, spi_slot, sensor_class=PAA5100):
        # spi_port=0, spi_slot='front' for the front sensor
        self.sensor = sensor_class(spi_port=spi_port, spi_cs=1,
                                   spi_cs_gpio=BG_CS_FRONT_BCM if spi_slot == 'front' else BG_CS_BACK_BCM)
        self.sensor.set_orientation(invert_x=True, invert_y=False, swap_xy=False)

        self.last_rel_t, self.last_rec = time.time(), 0
        self.rel_x, self.rel_y = 0, 0
        self.abs_x, self.abs_y = 0, 0

    def loop(self):
        try:
            x, y = self.sensor.get_motion(timeout=0.001)
            self.last_rec = time.time()
            self.rel_x += x
            self.rel_y += y
            self.abs_x += x
            self.abs_y += y
        except RuntimeError:
            pass

    def get_rel_motion(self, get_dt=False):
        ret_mot = np.array([self.rel_x, self.rel_y])
        self.rel_x = self.rel_y = 0  # reset

        dt = self.last_rec - self.last_rel_t
        self.last_rel_t = time.time()

        if get_dt:
            return ret_mot, dt
        return ret_mot

    def get_abs_motion(self):
        return np.array([self.abs_x, self.abs_y])


def _main():  # example code with OmniDrive

    from omni_drive import OmniDrive

    flo = MotionSensor(0, 'front')

    lin_act_pins = {'up': 22, 'down': 4, 'enable': 27}
    roller0_pins = {'right': 23, 'left': 24, 'pwm': 18}
    roller1_pins = {'right': 5, 'left': 6, 'pwm': 13}
    roller2_pins = {'right': 25, 'left': 26, 'pwm': 12}
    roller_pins = [roller0_pins, roller1_pins, roller2_pins]

    omni_drive = OmniDrive(roller_pins, lin_act_pins, up_trans_t=6, down_trans_t=6, pwm_freq=1000)
    omni_drive.setup()
    omni_drive.simple_drive('forward', t=10, blocking=False)

    cx = 0
    cy = 0
    c = 0
    tprev = 0

    try:
        while True:
            try:
                omni_drive.loop()
                flo.loop()
            except RuntimeError:
                continue

            tx, ty = flo.get_abs_motion()

            if c >= 150:
                cx, cy = flo.get_rel_motion()
                # print("Relative: cx {:03d} cy {:03d} | Absolute: x {:03d} y {:03d}".format(cx, cy, tx, ty))
                cx = cy = c = 0

            tnow = int(time.time())
            if tnow >= tprev + 1:
                print("{}->{}: relative: cx {:03d} cy {:03d} | Absolute: x {:03d} y {:03d}".format(tprev, tnow, cx, cy,
                                                                                                   tx, ty))
                tprev = tnow

            c += 1
            time.sleep(0.1)

    except KeyboardInterrupt:
        omni_drive.cleanup()


if __name__ == '__main__':
    _main()
