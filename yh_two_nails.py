#!/usr/bin/python

import subprocess
subprocess.run('sudo pigpiod', shell=True, check=True)
import pigpio
pi = pigpio.pi()
from zh_Utilities import landmarks, brickMap, hmRPYG, hmRPYP, poseTags, drawGroundOG, drawRigidBodyOG, keyboardCtrl, drawFloor, drawLineOG, robot, R, ser
import time
speed_0 = 1/2

if __name__ == "__main__":

    try:
        # init basics
        myRobot = robot(hmRPYG, None, None, ser=ser, config="dog3Config.json")
        # exit program.
        myRobot.resetPose()
        myRobot.two_nails_on_board()
        pi.stop()
        subprocess.run('sudo killall pigpiod', shell=True, check=True)
        print("Program Exited")
        

    except KeyboardInterrupt:
        myRobot.resetPose()
        myRobot.stopwalknew()
        pi.stop()
        subprocess.run('sudo killall pigpiod', shell=True, check=True)
        print("Program Exited")
