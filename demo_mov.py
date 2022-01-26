# Author: Alberto Martínez Rodríguez
# Date: January 2022
# 
# Make sure to have CoppeliaSim running, with followig scene loaded: tests.ttt

import time

import CoppeliaAPI.csimBrickpi as brickpi3

BP = brickpi3.BrickPi3()

# You have to associate motors with ports on the config file
BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) # reset encoder B
BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) # reset encoder C

try:
    while True:
        # The following BP.get_motor_encoder function returns the encoder value
        encoder_val1 = BP.get_motor_encoder(BP.PORT_B)
        encoder_val2 = BP.get_motor_encoder(BP.PORT_C)
        print("Encoder values: {}, {}".format(encoder_val1, encoder_val2))

        # set the target speed for motors in Degrees Per Second
        BP.set_motor_dps(BP.PORT_B , 200)
        BP.set_motor_dps(BP.PORT_C , 200)
        time.sleep(2)
        
        # You can set the speed of 2 motors at the same time
        BP.set_motor_dps(BP.PORT_B + BP.PORT_C , -100)
        time.sleep(2)
except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors.