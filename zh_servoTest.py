#!/usr/bin/python

import subprocess
subprocess.run('sudo pigpiod', shell=True, check=True)
import pigpio
pi = pigpio.pi()
from zh_Utilities import landmarks, brickMap, hmRPYG, hmRPYP, poseTags, drawGroundOG, drawRigidBodyOG, keyboardCtrl, drawFloor, drawLineOG, robot, R, ser
import time


if __name__ == "__main__":

    try:
        # init basics.
        myTags = landmarks(hmRPYP, poseTags, None)  # tags use RPYP pose. Not specifying the axes.
        myBrickMap = brickMap(hmRPYG, None)
        myRobot = robot(hmRPYG, None, poseTags, ser=ser, servoConfig="dog3ServoConfig.json")
        myRobot.bodyPose[-1] = myBrickMap.brickThickness + myRobot.initFeetPos[0][1] - myRobot.linkageFrameOffsets[0][-1]
        myRobot.feetPosControl(myRobot.initFeetPos)
        myRobot.propagateAllLegJointPoses()

        '''
        # adjust height.
        myRobot.adjustHeight(75)  # default height is 95
        time.sleep(2)
        
        myRobot.adjustHeight(95)
        time.sleep(2)
        '''

        # buzzer control.
        myRobot.buzzer(True)
        time.sleep(2.5)
        myRobot.buzzer(False)

        # servo control.
        myRobot.resetPose()
        time.sleep(1)

        # grasping the brick.
        # first lower the linkages.
        myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["linkageTilt"], 1/2)
        time.sleep(1)
        myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["linkageDown"], 1/2)
        time.sleep(1)
        myRobot.closeGripper()
        time.sleep(0.1)
        myRobot.openGripper()
        time.sleep(20)
        myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["linkageFlat"], 1/10)
        time.sleep(1)
        myRobot.singleServoCtrl(1, myRobot.servoCriticalAngles["brickFlat"], 1/10)
        time.sleep(1)
        myRobot.closeGripper()
        time.sleep(1)

        myRobot.buzzer(True)
        time.sleep(4.5)
        myRobot.buzzer(False)

        # pull out the brick.
        myRobot.adjustHeight(95)  # walk mode changed to trot.
        myRobot.changeclearance()  # will walk.
        myRobot.stopwalknew()  # change mode to triangular walk.
        myRobot.triangularwalk(0, 40, continuous=False)
        time.sleep(2)
        myRobot.triangularwalk(0, 40, continuous=False)
        time.sleep(2)
        myRobot.triangularwalk(0, 40, continuous=False)
        time.sleep(2)
        '''
        myRobot.triangularwalk(0, 40, continuous=False)
        time.sleep(2)
        '''

        # place brick.
        myRobot.placeBrickPhase1()
        myRobot.pushBrick(0, verbose=True)
        myRobot.pushBrick(32, verbose=True)  # walk mode changed to trot.
        myRobot.placeBrickPhase2()
        myRobot.leanBack(32, verbose=True)  # walk mode changed to trot.
        # myRobot.closeGripper()
        # myRobot.openGripper()
        myRobot.placeBrickPhase3()
        myRobot.placeBrickPhase4()
        time.sleep(1)

        '''
        # change walk mode to triangular walk.
        myRobot.changeclearance(20)
        myRobot.stopwalknew()
        '''

        # exit program.
        myRobot.resetPose()
        pi.stop()
        subprocess.Popen(["sudo", "killall", "pigpiod"])
        print("Program Exited")

    except KeyboardInterrupt:
        myRobot.resetPose()
        myRobot.stopwalknew()
        pi.stop()
        subprocess.Popen(["sudo", "killall", "pigpiod"])
        print("Program Exited")
