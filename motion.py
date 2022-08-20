import time
import argparse
from pmw3901 import PMW3901, PAA5100, BG_CS_FRONT_BCM, BG_CS_BACK_BCM


def get_front_motion_sensor():
    SensorClass = PAA5100
    spi_slot = 'front'

    flo = SensorClass(spi_port=0, spi_cs=1, spi_cs_gpio=BG_CS_FRONT_BCM if spi_slot == 'front' else BG_CS_BACK_BCM)
    flo.set_orientation(invert_x=True, invert_y=False, swap_xy=False)

    return flo


def get_side_motion_sensor():
    pass  # TODO


def _main():
    from omni_drive import OmniDrive

    flo = get_front_motion_sensor()

    lin_act_pins = {'up': 22, 'down': 4, 'enable': 27}
    roller0_pins = {'right': 23, 'left': 24, 'pwm': 18}
    roller1_pins = {'right': 5, 'left': 6, 'pwm': 13}
    roller2_pins = {'right': 25, 'left': 26, 'pwm': 12}
    roller_pins = [roller0_pins, roller1_pins, roller2_pins]

    omni_drive = OmniDrive(roller_pins, lin_act_pins, up_trans_t=6, down_trans_t=6, pwm_freq=1000)
    omni_drive.setup()
    omni_drive.simple_drive('forward', t=10, blocking=False)

    tx = 0
    ty = 0

    cx = 0
    cy = 0
    c = 0

    tprev = 0

    try:
        while True:
            try:
                x, y = flo.get_motion()
            except RuntimeError:
                continue
            tx += x
            ty += y

            cx += x
            cy += y
            if c >= 150:
                # print("Relative: cx {:03d} cy {:03d} | Absolute: x {:03d} y {:03d}".format(cx, cy, tx, ty))
                cx = cy = c = 0

            tnow = int(time.time())
            if tnow >= tprev + 1:
                print("{}->{}: relative: cx {:03d} cy {:03d} | Absolute: x {:03d} y {:03d}".format(tprev, tnow, cx, cy,
                                                                                                   tx, ty))
                tprev = tnow

            c += 1
            time.sleep(0.1)
            omni_drive.loop()

    except KeyboardInterrupt:
        omni_drive.cleanup()


if __name__ == '__main__':
    _main()
