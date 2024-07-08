#!/usr/bin/python

import subprocess
subprocess.run('sudo pigpiod', shell=True, check=True)
import pigpio
pi = pigpio.pi()
from zh_Utilities import landmarks, brickMap, hmRPYG, hmRPYP, poseTags, drawGroundOG, drawRigidBodyOG, keyboardCtrl, drawFloor, drawLineOG, robot, R
import time

if __name__ == "__main__":

    try:
        # init basics.
        myTags = landmarks(hmRPYP, poseTags, None)  # tags use RPYP pose. Not specifying the axes.
        myBrickMap = brickMap(hmRPYG, None)
        myRobot = robot(hmRPYG, None, poseTags, servoConfig="dog2ServoConfig.json")
        myRobot.bodyPose[-1] = myBrickMap.brickThickness + myRobot.initFeetPos[0][1] - myRobot.linkageFrameOffsets[0][-1]
        myRobot.feetPosControl(myRobot.initFeetPos)
        myRobot.propagateAllLegJointPoses()
        # servo control.
        myRobot.resetPose()
        myRobot.openGripper()
        time.sleep(3)
        myRobot.closeGripper()
        myRobot.placeBrickPhase1()
        myRobot.pushBrick(10, verbose=True)
        myRobot.placeBrickPhase2()
        myRobot.placeBrickPhase3()
        time.sleep(1)
        # exit program.
        myRobot.resetPose()
        pi.stop()
        subprocess.Popen(["sudo", "killall", "pigpiod"])
        print("Program Exited")

    except KeyboardInterrupt:
        myRobot.resetPose()
        pi.stop()
        subprocess.Popen(["sudo", "killall", "pigpiod"])
        print("Program Exited")
