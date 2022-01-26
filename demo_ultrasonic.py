# Author: Alberto Martínez Rodríguez
# Date: January 2022
# 
# Make sure to have CoppeliaSim running, with followig scene loaded: tests.ttt

import time

import CoppeliaAPI.csimBrickpi as brickpi3

BP = brickpi3.BrickPi3()
BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.EV3_ULTRASONIC_CM)
# To use a second sensor, you have to set the ALT_ULTRASOUND_DIR and ALT_ULTRASOUND_PORT on the config file
BP.set_sensor_type(BP.PORT_2, BP.SENSOR_TYPE.EV3_ULTRASONIC_CM)

try:
    while True:
        value1 = BP.get_sensor(BP.PORT_1)
        value2 = BP.get_sensor(BP.PORT_2)
        print("Front Ultrasonic value: {}".format(value1))
        print("Left Ultrasonic value: {}\n".format(value2))
        time.sleep(1)
except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors.