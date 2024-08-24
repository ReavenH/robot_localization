from zh_Utilities import hmRPYG, robot, ser
import cv2
import time
import numpy as np
import os
import serial
from scipy.io import savemat
import subprocess
import pigpio
pi = pigpio.pi()

os.system("sudo systemctl stop serial-getty@ttyS0.service")

myRobot = robot(hmRPYG, None, None, ser, config="dog3Config.json", vidsrc=1)
myRobot.initBottomCamera()

timePrev = time.time()
yawTolerance = 4  # default 5
yawThreshold = 8
offsetTolerance = 100
isturn = False
accCentroid = np.array([0.0, 0.0])
accCentroidCount = 0

minFreeturnDeg = 8
freeturnCount = 0

sleepTime = 0.5

flagHistory = np.array([])  # only for tests, the atCrossing flag's history.
myRobot.startwalknew()
myRobot.triangularwalk(0, distance=20)

walkDis = 35

while myRobot.cap.isOpened:
    try:
        myRobot.readGlobalStep()
        # print("global step:", myRobot.globalStep)
        ret = myRobot.getPoseFromCircles(verbose=False, minCircles=5, display=False)
        if ret == None:  # stop the robot if the camera is down.
            myRobot.currentAction = 'S'
        flagHistory = np.append(flagHistory, (len(myRobot.bottomLineYaw) == 2))
        print("isMoving: {} | prevCrossing: {} | atCrossing: {} | currentAction: {} | countCrossing: {}".format(myRobot.isMoving, myRobot.prevCrossing, myRobot.atCrossing, myRobot.currentAction, myRobot.countCrossing))
        
        if myRobot.currentAction == 'S':
            # myRobot.stopwalknew()
            # print("Action Stopped (currentAction is 'S')")
            myRobot.interrupt()
            print("Action Interrputed (currentAction is 'S')")
            break

        elif myRobot.currentAction == 'L':
            myRobot.interrupt()
            time.sleep(0.7)
            # blind turn, wait until globalStep is greater than 1.0
            myRobot.freeturn(20)
            for i in range(4): # default 5
                myRobot.waitGlobalStep()
                myRobot.freeturn(20) # default 3zh_planarWalk1.py0
                # time.sleep(2.5)

            myRobot.countCrossing += 1
            myRobot.currentAction = 'F'

        elif myRobot.currentAction == 'R':
            myRobot.interrupt()
            print("Action Interrputed (currentAction is 'R')")
            break

        elif myRobot.currentAction == "G":  
            # grab the brick from docker.
            print("Grabing the brick from docker.")
            myRobot.interrupt()
            
            # TODO: use rpyPID control to align the body.

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
            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["linkageDown"], 1/10)
            time.sleep(1)
            myRobot.closeGripper()
            time.sleep(0.1)
            myRobot.openGripper()
            time.sleep(20)  # adjustable. 27 default.
            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["linkageFlat"], 1/10)
            time.sleep(1)
            myRobot.singleServoCtrl(1, myRobot.servoCriticalAngles["brickFlat"], 1/10)
            time.sleep(1)
            myRobot.closeGripper()
            time.sleep(1)

            myRobot.buzzer(True)
            time.sleep(3)  # default 4.5
            myRobot.buzzer(False)

            # pull out the brick.
            myRobot.currentAction = 'F'
            myRobot.countCrossing += 1
            myRobot.triangularwalk(0, distance=walkDis, continuous=True)
            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles['linkageUp'], 1/10)
            # myRobot.globalStep = 1

            '''
            myRobot.adjustHeight(95)  # walk mode changed to trot.
            myRobot.changeclearance()  # will walk.
            myRobot.stopwalknew()  # change mode to triangular walk.
            '''

            '''
            myRobot.triangularwalk(0, 40, continuous=False)
            time.sleep(2)
            myRobot.triangularwalk(0, 40, continuous=False)
            time.sleep(2)
            myRobot.triangularwalk(0, 40, continuous=False)
            time.sleep(2)
            '''

            '''
            myRobot.triangularwalk(0, 40, continuous=False)
            time.sleep(2)
            '''

        elif myRobot.globalStep >= 1.0:

            if myRobot.currentAction == 'F' or myRobot.currentAction == 'C':
                
                # turn on IMU before climbing.
                if myRobot.currentAction == "C" and myRobot.isCounting == True:
                    '''
                    myRobot.interrupt()
                    
                    myRobot.adjustHeight(85)
                    myRobot.interrupt()
                    
                    '''
                    myRobot.changeclearance(val=30)
                    myRobot.switchIMU(True)
                    walkDis = 50
                    myRobot.isCounting = False

                elif myRobot.currentAction == "F" and myRobot.prevAction == "C":
                    '''
                    myRobot.interrupt()
                     
                    myRobot.adjustHeight(95)
                    myRobot.interrupt()
                    myRobot.changeclearance(val=30)
                    '''
                    myRobot.changeclearance(val=30)
                    myRobot.switchIMU(False)
                    walkDis = 40  # default 35

                if myRobot.lostVision in [0, 2, -2]:

                    yaw = myRobot.bottomLineYawStraight
                    print("centroid: [{:.2f}, {:.2f}] | atCrossing: {} |yaw(s): {}".format(myRobot.bottomLineCentroid[0], myRobot.bottomLineCentroid[1], myRobot.atCrossing, yaw))

                    isturn = False
                    if np.abs(yaw) > yawThreshold:
                        isturn = True

                    # control the robot.
                    if isturn and myRobot.lostVision == 0:
                        # stopwalknew()
                        myRobot.rlbPID()
                        print("RLB: {}".format(myRobot.RLB))
                        myRobot.rlbControl(myRobot.RLB)
                        myRobot.triangularwalk(myRobot.walkDir.copy(), distance=walkDis)
                    # tune the params.
                    elif myRobot.lostVision in [-2, 2]:
                        print("into SL")
                        if myRobot.lostVision == 2:
                            print("Slightly lost vision on the right.")
                            myRobot.triangularwalk(-25, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (walkDis - 20) + 20))
                            # continue
                        elif myRobot.lostVision == -2:
                            print("Slightly lost vision on the left.")
                            myRobot.triangularwalk(15, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (walkDis - 20) + 20))
                            # continue
                    else:
                        myRobot.triangularwalk(myRobot.walkDir.copy(), distance=walkDis)

                else:
                    # myRobot.startwalknew()
                    if myRobot.lostVision == 1:
                        print("Lost vision on the right.")
                        myRobot.triangularwalk(-30, 35)  # default 25
                    elif myRobot.lostVision == -1:
                        print("Lost vision on the left.")
                        myRobot.triangularwalk(30, 35)    

            elif myRobot.currentAction == "P":
                # place the brick.
                myRobot.interrupt()
                time.sleep(1.5)
                # adjust the yaw offset roughly.
                for i in range(12):
                    myRobot.getPoseFromCircles()
                while abs(myRobot.bottomLineYawStraight) > 8:
                    if myRobot.bottomLineYawStraight > 0:
                        myRobot.freeturn(max(min(myRobot.bottomLineYawStraight*1.5,-7.5),-15))
                        time.sleep(1.2)
                    elif myRobot.bottomLineYawStraight < 0:
                        myRobot.freeturn(min(max(myRobot.bottomLineYawStraight*1.5,7.5),15))
                        time.sleep(1.2)
                    for i in range(12):
                        myRobot.getPoseFromCircles()
                # adjust the lateral offset.
                for i in range(12):
                    myRobot.getPoseFromCircles()
                while myRobot.lostVision != 0:
                    print("Adjusting lateral offset: lostVision is {}".format(myRobot.lostVision))
                    if myRobot.lostVision > 0:
                        myRobot.triangularwalk(-90, 12, continuous=False)
                        myRobot.waitGlobalStep()
                    elif myRobot.lostVision < 0:
                        myRobot.triangularwalk(90, 12, continuous=False)
                        myRobot.waitGlobalStep()
                    for i in range(12):
                        myRobot.getPoseFromCircles()
                # use the RPY PID control from the ESP32 to rotate body without moving feet.
                myRobot.brickAlign()
                time.sleep(1.5)
                myRobot.rpyPID(aim = -2)
                myRobot.placeBrickPhase1()
                time.sleep(1)
                myRobot.pushBrick(-32, verbose = True)  # default -32
                time.sleep(4)
                myRobot.rpyPID(aim = -2)
                myRobot.pushBrick(32, verbose = True)  # default 32
                time.sleep(4)
                myRobot.placeBrickPhase2()
                myRobot.placeBrickPhase3()
                myRobot.placeBrickPhase4()
                myRobot.stopwalknew()
                myRobot.changeclearance()  # default 30
                myRobot.RPYCtl('yaw', 0)
                break

        # print("-------------------------------------------------------------------")

    except KeyboardInterrupt:
        # myRobot.stopwalknew()
        myRobot.interrupt()
        myRobot.resetPose()
        pi.stop()
        subprocess.Popen(["sudo", "killall", "pigpiod"])
        myRobot.switchIMU(False) 
        myRobot.ser.close()
        break

    except serial.SerialTimeoutException:
        # myRobot.stopwalknew()
        myRobot.interrupt()
        myRobot.resetPose()
        pi.stop()
        subprocess.Popen(["sudo", "killall", "pigpiod"])
        myRobot.switchIMU(False) 
        myRobot.ser.close()
        print("Serial Timeout, exited.")
        break

    except NameError:
        myRobot.interrupt()
        myRobot.resetPose()
        pi.stop()
        subprocess.Popen(["sudo", "killall", "pigpiod"])
        myRobot.switchIMU(False) 
        myRobot.ser.close()
        break

# myRobot.changeclearance()
# myRobot.triangularwalk(0, 0, continuous=False)
myRobot.interrupt()
# myRobot.waitGlobalStep()
myRobot.resetPose()
pi.stop()
subprocess.Popen(["sudo", "killall", "pigpiod"])
myRobot.switchIMU(False) 
myRobot.ser.close()
myRobot.cap.release()
cv2.destroyAllWindows()
exit()
