from zh_Utilities import hmRPYG, robot, ser
import cv2
import time
import numpy as np
import os
import serial
from scipy.io import savemat

'''
TODO:
Add visual aid when turning.
'''

os.system("sudo systemctl stop serial-getty@ttyS0.service")

myRobot = robot(hmRPYG, None, None, ser, config="dog3Config.json", vidsrc=0)
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

myRobot.startwalknew()
myRobot.triangularwalk(0, distance=20)

while myRobot.cap.isOpened:
    try:
        myRobot.readGlobalStep()
        # print("global step:", myRobot.globalStep)
        ret = myRobot.getPoseFromCircles(verbose=False, minCircles=5, display=False)
        if ret == None:  # stop the robot if the camera is down.
            myRobot.currentAction = 'S'
        # currentAction = myRobot.schedular()
        # print("currentAction: {}".format(currentAction))
        # print("myRobot.globalStep: {}".format(myRobot.globalStep))
        # print("atCrossing: {} | FIFO: {}".format(myRobot.atCrossing, list(myRobot.atCrossingFIFO)))
        print("isMoving: {} | prevCrossing: {} | atCrossing: {} | currentAction: {} | countCrossing: {}".format(myRobot.isMoving, myRobot.prevCrossing, myRobot.atCrossing, myRobot.currentAction, myRobot.countCrossing))
        
        if myRobot.currentAction == 'S':
            myRobot.interrupt()
            print("Action Interrputed (currentAction is 'S')")
            break

        elif myRobot.currentAction == 'L':
            myRobot.interrupt()
            time.sleep(0.7)
            # rotate with visual aid: keep the manhattan centroid of the STRAIGHT line in the center.
            # currently only adjusting the LR direction.
            myRobot.getPoseFromCircles()
            print("Lost Vision is [{}] | countCrossing is [{}] | lenYaw is [{}]".format(myRobot.lostVision, myRobot.countCrossing, len(myRobot.bottomLineYaw)))


            while myRobot.lostVision != 0:
                if myRobot.lostVision > 0:
                    # lost vision / slightly lost vision on the right.
                    myRobot.triangularwalk(-90, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (10 - 5) + 5), continuous=False)
                    myRobot.waitGlobalStep()
                elif myRobot.lostVision < 0:
                    myRobot.triangularwalk(90, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (10 - 5) + 5), continuous=False)
                    myRobot.waitGlobalStep()
                myRobot.getPoseFromCircles()
                print("Lost Vision is [{}] | countCrossing is [{}] | lenYaw is [{}]".format(myRobot.lostVision, myRobot.countCrossing, len(myRobot.bottomLineYaw)))

            myRobot.freeturn(20)
            myRobot.waitGlobalStep()

            for i in range(5):
                myRobot.getPoseFromCircles()
                print("Lost Vision is [{}]".format(myRobot.lostVision))
                while myRobot.lostVision != 0:
                    if myRobot.lostVision > 0:
                        # lost vision / slightly lost vision on the right.
                        myRobot.triangularwalk(-90, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (10 - 5) + 5), continuous=False)
                        myRobot.waitGlobalStep()
                    elif myRobot.lostVision < 0:
                        myRobot.triangularwalk(90, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (10 - 5) + 5), continuous=False)
                        myRobot.waitGlobalStep()
                    myRobot.getPoseFromCircles()
                    print("Lost Vision is [{}] | countCrossing is [{}] | lenYaw is [{}]".format(myRobot.lostVision, myRobot.countCrossing, len(myRobot.bottomLineYaw)))
                myRobot.freeturn(20)
                myRobot.waitGlobalStep()

            myRobot.countCrossing += 1
            myRobot.currentAction = 'F'

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
                        myRobot.rlbPID()
                        print("RLB: {}".format(myRobot.RLB))
                        myRobot.rlbControl(myRobot.RLB)
                        myRobot.triangularwalk(myRobot.walkDir.copy())
                    # tune the params.
                    elif myRobot.lostVision in [-2, 2]:
                        print("into SL")
                        if myRobot.lostVision == 2:
                            print("Slightly lost vision on the right.")
                            myRobot.triangularwalk(-25, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (30 - 20) + 20))
                            # continue
                        elif myRobot.lostVision == -2:
                            print("Slightly lost vision on the left.")
                            myRobot.triangularwalk(15, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (30 - 20) + 20))
                            # continue
                    else:
                        myRobot.triangularwalk(myRobot.walkDir.copy())
                else:
                    # myRobot.startwalknew()
                    if myRobot.lostVision == 1:
                        print("Lost vision on the right.")
                        myRobot.triangularwalk(-30, 25)
                    elif myRobot.lostVision == -1:
                        print("Lost vision on the left.")
                        myRobot.triangularwalk(30, 25)       
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

myRobot.interrupt()
myRobot.ser.close()
myRobot.cap.release()
cv2.destroyAllWindows()
exit()
