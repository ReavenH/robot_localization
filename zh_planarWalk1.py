from zh_Utilities import hmRPYG, robot, ser
import cv2
import time
import numpy as np
import os
import serial
from scipy.io import savemat

'''
TODO:
1. Separate the system into schedular and the implementation part.
2. Use FBLR to represent the direction to go, use S to represent stop.
3. input of the schedular: the number of passed crossings -> to search the action to take at the current crossing.
4. input of the motionController: the alphabet of the motion to take.
'''

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

'''
myRobot.adjustHeight(105)
while True:
    myRobot.readGlobalStep()
    if myRobot.globalStep >= 1.0:
        break
    else:
        continue
'''

'''
myRobot.buzzer(True)
time.sleep(0.5)
myRobot.buzzer(False)
myRobot.resetPose()
time.sleep(2)
myRobot.closeGripper()
'''

myRobot.startwalknew()
myRobot.triangularwalk(0, distance=20)

while myRobot.cap.isOpened:
    try:
        myRobot.readGlobalStep()
        # print("global step:", myRobot.globalStep)
        myRobot.getPoseFromCircles(verbose=False, minCircles=5, display=False)
        flagHistory = np.append(flagHistory, (len(myRobot.bottomLineYaw) == 2))
        # currentAction = myRobot.schedular()
        # print("currentAction: {}".format(currentAction))
        # print("myRobot.globalStep: {}".format(myRobot.globalStep))
        # print("atCrossing: {} | FIFO: {}".format(myRobot.atCrossing, list(myRobot.atCrossingFIFO)))
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
            # myRobot.triangularwalk(0, -30, continuous=False)
            # adjust the pose before turning.
            # myRobot.getPoseFromCircles()
            # blind turn, wait until globalStep is greater than 1.0
            myRobot.freeturn(30)
            for i in range(4):
                while True:
                    myRobot.readGlobalStep()
                    if myRobot.globalStep >= 1.0:
                        break
                    else:
                        continue
                myRobot.freeturn(35) # default 30
                # time.sleep(2.5)
            '''
            # translate to the left for 3 steps.
            for i in range(2):
                while True:
                    myRobot.readGlobalStep()
                    if myRobot.globalStep >= 1.0:
                        break
                    else:
                        continue
                myRobot.triangularwalk(90, 30, continuous=False)
                time.sleep(2.5)
            '''
            myRobot.countCrossing += 1
            myRobot.currentAction = 'F'
            '''
            while True:
                myRobot.getPoseFromCircles()
                print("Action L: Yaw is {}".format(myRobot.bottomLineYawStraight))
                if abs(myRobot.bottomLineYawStraight) < 10:
                    myRobot.countCrossing += 1
                    break
                else:
                    while True:
                        myRobot.readGlobalStep()
                        if myRobot.globalStep >= 1.0:
                            break
                    myRobot.freeturn(30)
                    time.sleep(2.5)  # wait until finish.
            '''

        elif myRobot.currentAction == 'R':
            myRobot.interrupt()
            print("Action Interrputed (currentAction is 'R')")
            break

        elif myRobot.globalStep >= 1.0:

            # centroid = accCentroid / accCentroidCount
            # accCentroidCount = 0
            # accCentroid = np.array([0.0, 0.0])
            if myRobot.currentAction == 'F':
                if myRobot.lostVision in [0, 2, -2]:

                    yaw = myRobot.bottomLineYawStraight
                    print("centroid: [{:.2f}, {:.2f}] | atCrossing: {} |yaw(s): {}".format(myRobot.bottomLineCentroid[0], myRobot.bottomLineCentroid[1], myRobot.atCrossing, yaw))

                    isturn = False
                    if np.abs(yaw) > yawThreshold:
                        isturn = True

                    # control the robot.
                    if isturn and myRobot.lostVision == 0:
                        # stopwalknew()
                        if yaw < -yawTolerance:  # turn to the left.
                            myRobot.freeturn(min(max(yaw*1.5,7.5),15))  # no more than 15, no less than 7.5
                        elif yaw > yawTolerance:
                            myRobot.freeturn(max(min(yaw*1.5,-7.5),-15))  # no less than -15

                    # tune the params.
                    elif myRobot.lostVision in [-2, 2]:
                        print("into SL")
                        if myRobot.lostVision == 2:
                            print("Slightly lost vision on the right.")
                            myRobot.triangularwalk(-25, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (45 - 20) + 20))
                            # continue
                        elif myRobot.lostVision == -2:
                            print("Slightly lost vision on the left.")
                            myRobot.triangularwalk(15, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (45 - 20) + 20))
                            # continue
                    else:
                        myRobot.triangularwalk(myRobot.walkDir.copy())
                else:
                    # myRobot.startwalknew()
                    if myRobot.lostVision == 1:
                        print("Lost vision on the right.")
                        # myRobot.stopwalknew()
                        # myRobot.triangularwalk(-90, 20, continuous=False)  # 50 degs for discrete, 30 may be fine for a continuous walk.
                        # time.sleep(0.5)
                        myRobot.triangularwalk(-30, 30)
                    elif myRobot.lostVision == -1:
                        print("Lost vision on the left.")
                        # myRobot.stopwalknew()
                        # myRobot.triangularwalk(90, 20, continuous=False)
                        # time.sleep(0.5)
                        myRobot.triangularwalk(30, 30)       
        # print("-------------------------------------------------------------------")

    except KeyboardInterrupt:
        # myRobot.stopwalknew()
        myRobot.interrupt()
        myRobot.ser.close()
        break

    except serial.SerialTimeoutException:
        # myRobot.stopwalknew()
        myRobot.interrupt()
        myRobot.ser.close()
        print("Serial Timeout, exited.")
        break

# savemat("zh_flagHistory.mat", {"data": flagHistory})
np.savetxt("zh_flagHistory.txt", flagHistory, fmt = '%d')
myRobot.cap.release()
cv2.destroyAllWindows()
exit()
