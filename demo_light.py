import time

import CoppeliaAPI.csimBrickpi as brickpi3

BP = brickpi3.BrickPi3()
BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.NXT_LIGHT_OFF)

try:
    while True:
        # Get the light value
        value = BP.get_sensor(BP.PORT_1)
        print("Light Value: {}".format(value))
        time.sleep(1)
except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors.