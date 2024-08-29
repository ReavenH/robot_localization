#!/usr/bin/python

import subprocess
subprocess.run('sudo pigpiod', shell=True, check=True)
import pigpio
pi = pigpio.pi()
from zh_Utilities import landmarks, brickMap, hmRPYG, hmRPYP, poseTags, drawGroundOG, drawRigidBodyOG, keyboardCtrl, drawFloor, drawLineOG, robot, R, ser
import time
cnt=0
speed_0 = 1/2

if __name__ == "__main__":

    try:
        while(cnt<1):
            # init basics.
            myTags = landmarks(hmRPYP, poseTags, None)  # tags use RPYP pose. Not specifying the axes.
            myBrickMap = brickMap(hmRPYG, None)
            myRobot = robot(hmRPYG, None, poseTags, ser=ser, config="dog3ServoConfig.json")
            myRobot.bodyPose[-1] = myBrickMap.brickThickness + myRobot.initFeetPos[0][1] - myRobot.linkageFrameOffsets[0][-1]
            myRobot.feetPosControl(myRobot.initFeetPos)
            myRobot.propagateAllLegJointPoses()
            
            # buzzer control.
            myRobot.buzzer(True)
            time.sleep(0.5)
            myRobot.buzzer(False)

            # servo control.
            # myRobot.resetPose()
            # time.sleep(1)

            # grasping the brick.
            # put down two nails
            myRobot.adjustHeight(95)
            time.sleep(1)
            myRobot.singleServoCtrl(1, myRobot.servoCriticalAngles["gripperAdjustment1"], 1/10)#need change 950
            time.sleep(1)
            myRobot.singleServoCtrl(2, myRobot.servoCriticalAngles["gripperAdjustment2"], 1/10)
            time.sleep(3)
            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["linkageDown1"], speed_0)
            time.sleep(1)
            # myRobot.singleServoCtrl(0, 1000, speed_0)
            # time.sleep(1)
            # myRobot.singleServoCtrl(0, 1300, speed_0)
            # time.sleep(1)
            myRobot.singleServoCtrl(1, myRobot.servoCriticalAngles["linkageAdjustment1"], 1/10)#need change 950
            time.sleep(1)
            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["brickDown1"], speed_0/10)#need change
            time.sleep(1)


            myRobot.singleServoCtrl(2, myRobot.servoCriticalAngles["gripperLoose"], 1 / 2)
            time.sleep(2)
            # myRobot.singleServoCtrl(2, 2200, 1 / 10)
            # time.sleep(1)
            # myRobot.singleServoCtrl(2, 2500, 1 / 10)
            # time.sleep(1)
            myRobot.singleServoCtrl(2, myRobot.servoCriticalAngles["gripperAdjustment2"], 1/20)# change to 500
            time.sleep(2)

            # myRobot.singleServoCtrl(1, 900, 1/10)#need change 950
            # time.sleep(1)
            # myRobot.pushBrick(-5)
            # time.sleep(1)

            myRobot.adjustHeight(90,dis=0.5)
            time.sleep(2)
            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["servoAdjustment1"], speed_0)
            time.sleep(1)


            
            myRobot.adjustHeight(80,dis=0.5)
            time.sleep(2)
            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["servoAdjustment2"], speed_0)
            time.sleep(1)
            
            # myRobot.singleServoCtrl(1, 800, 1/10)#need change 950
            # time.sleep(1)
            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["servoAdjustment1"], speed_0)
            time.sleep(1)
            myRobot.adjustHeight(90)
            time.sleep(1)
            myRobot.singleServoCtrl(2, myRobot.servoCriticalAngles["gripperAdjustment2"], 1/10)
            time.sleep(1)

            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["servoAdjustment3"], speed_0)
            time.sleep(1)
            myRobot.adjustHeight(100)
            time.sleep(1)
            myRobot.singleServoCtrl(2, myRobot.servoCriticalAngles["gripperAdjustment2"], 1/10)
            time.sleep(1)

            myRobot.adjustHeight(110)
            time.sleep(1)
            myRobot.singleServoCtrl(2, myRobot.servoCriticalAngles["gripperAdjustment2"], 1/10)
            time.sleep(1)

            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["servoAdjustment3"], speed_0/10)
            time.sleep(1)
            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["gripperAdjustment2"], speed_0/20)
            time.sleep(3)
            # myRobot.singleServoCtrl(1, 550, 1/10)#need change 950
            # time.sleep(1)
            
            
            #pin down two nails
            myRobot.pushBrick(-25)
            myRobot.singleServoCtrl(1, myRobot.servoCriticalAngles["pinDownAdjustment"], 1/10)
            time.sleep(1)
            myRobot.adjustHeight(80)
            time.sleep(5)
            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["pinDownPWM1"], 1/2)
            time.sleep(1)
            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["pinDownPWM2"], 1/8)
            time.sleep(2)
            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["pinDownPWM1"], 1/2)
            time.sleep(1)
            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["pinDownPWM2"], 1/8)
            time.sleep(2)
            myRobot.pushBrick(25)
            
            
            # return to the normal state
            myRobot.adjustHeight(110)
            time.sleep(3)
            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["pinDownPWM2"], 1 / 10)
            time.sleep(1)
            
            
            cnt+=1
            
        # exit program.
        myRobot.resetPose()
        pi.stop()
        subprocess.run('sudo killall pigpiod', shell=True, check=True)
        print("Program Exited")
        

    except KeyboardInterrupt:
        myRobot.resetPose()
        myRobot.stopwalknew()
        pi.stop()
        subprocess.run('sudo killall pigpiod', shell=True, check=True)
        print("Program Exited")
