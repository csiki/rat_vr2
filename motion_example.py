import time
import argparse
from pmw3901 import PMW3901, PAA5100, BG_CS_FRONT_BCM, BG_CS_BACK_BCM


print("""motion.py - Detect flow/motion in front of the PMW3901 sensor.

Press Ctrl+C to exit!
""")


# Pick the right class for the specified breakout
SensorClass = PAA5100
spi_slot = 'front'
rotation = 180

flo = SensorClass(spi_port=0, spi_cs=1, spi_cs_gpio=BG_CS_FRONT_BCM if spi_slot == 'front' else BG_CS_BACK_BCM)
flo.set_rotation(rotation)

tx = 0
ty = 0

cx = 0
cy = 0
c = 0

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
            print("Relative: cx {:03d} cy {:03d} | Absolute: x {:03d} y {:03d}".format(cx, cy, tx, ty))
            cx = cy = c = 0
        c += 1
        time.sleep(0.01)
except KeyboardInterrupt:
    pass
