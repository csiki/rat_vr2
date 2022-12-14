
FRONT_MOTION_PARAMS = dict(spi_port=0, spi_cs=1, spi_slot=0, swap_xy=True, invert_x=False, invert_y=False)
SIDE_MOTION_PARAMS = dict(spi_port=1, spi_cs=0, spi_slot=1, swap_xy=True, invert_x=False, invert_y=False)
# front, then side sensor mapping; first int is motion axis index of a sensor, second is the 3 DoF axis index
MOTION_SENSORS_AXIS_MAPPING = [((0, 0), (1, 2)), ((0, 1), (1, 2))]

PRESSURE_SETPOINT = 94.4
VALVE_MS_PER_UL = 0.5  # TODO calc
