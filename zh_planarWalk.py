from zh_Utilities import hmRPYG, robot, ser
import cv2
import time
import numpy as np
import os
import serial
'''
TODO:
1. DONE. Use the Manhattan centroid as the point for distance offset (this ensures that the point is an existing circle).
2. Give speeds based on the robot's dynamic state (still or moving).
3. Use larger circles to decide the next action, thus to navigate on a plane.
4. Localize on a 2D structure.
'''

'''
triangular walk: 
    90 deg: min dis is 10
    -90 deg: min dis is 13 (should be -73 degs which will practically translate.)

freeturn:
    + min deg: 8
    _ min deg: -8
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

sleepTime = 0.5

myRobot.startwalknew()
myRobot.triangularwalk(0)

while myRobot.cap.isOpened:
    try:
        myRobot.readGlobalStep()
        # print("global step:", myRobot.globalStep)
        myRobot.getPoseFromCircles(verbose=False, minCircles=5, display=False)
        # currentAction = myRobot.schedular()
        # print("currentAction: {}".format(currentAction))
        # print("myRobot.globalStep: {}".format(myRobot.globalStep))
        # print("atCrossing: {} | FIFO: {}".format(myRobot.atCrossing, list(myRobot.atCrossingFIFO)))
        print("isMoving: {} | prevCrossing: {} | atCrossing: {} | currentAction: {} | countCrossing: {}".format(myRobot.isMoving, myRobot.prevCrossing, myRobot.atCrossing, myRobot.currentAction, myRobot.countCrossing))
        '''
        if myRobot.globalStep < 1.0:
            accCentroid += myRobot.bottomLineCentroid  # only calculates the vertical offset.
            accCentroidCount += 1
        '''

        # elif myRobot.globalStep >= 1.0:
        # else:
        if myRobot.globalStep >= 1.0:

            # centroid = accCentroid / accCentroidCount
            # accCentroidCount = 0
            # accCentroid = np.array([0.0, 0.0])

            if myRobot.lostVision in [0, 2, -2]:
                '''
                yaw = - myRobot.bottomLineYaw

                
                # process yaw to output a correct datatype.
                if len(yaw) == 2:
                    yaw = yaw[np.argmin(np.abs(yaw))]
                    yaw = np.sign(yaw) * np.abs(yaw)

                if isinstance(yaw, (int, float)) != True:
                    yaw = yaw[0]
                '''
                yaw = myRobot.bottomLineYawStraight
                print("centroid: [{:.2f}, {:.2f}] | atCrossing: {} |yaw(s): {}".format(myRobot.bottomLineCentroid[0], myRobot.bottomLineCentroid[1], myRobot.atCrossing, yaw))

                isturn = False
                if np.abs(yaw) > yawThreshold:
                    isturn = True

                # control the robot.
                if isturn and myRobot.lostVision == 0:
                    # stopwalknew()
                    '''
                    if np.abs(myRobot.bottomLineCentroid[1]) > offsetTolerance:
                        print("111")
                        if myRobot.bottomLineCentroid[1] < 0:
                            print("Walk to the right before turning.")
                            myRobot.stopwalknew()
                            myRobot.triangularwalk(-90, 30, continuous=False)
                        if myRobot.bottomLineCentroid[1] > 0:
                            print("Walk to the left before turning.")
                            myRobot.stopwalknew()
                            myRobot.triangularwalk(90, 30, continuous=False)
                    '''
                    if yaw < -yawTolerance:  # turn to the left.
                        myRobot.freeturn(min(max(yaw*1.5,7.5),15))  # no more than 15, no less than 7.5
                    elif yaw > yawTolerance:
                        myRobot.freeturn(max(min(yaw*1.5,-7.5),-15))  # no less than -15

                # tune the params.
                elif myRobot.lostVision in [-2, 2]:
                    print("into SL")
                    if myRobot.lostVision == 2:
                        print("Slightly lost vision on the right.")
                        myRobot.triangularwalk(-15, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (45 - 20) + 20))
                        # continue
                    elif myRobot.lostVision == -2:
                        print("Slightly lost vision on the left.")
                        myRobot.triangularwalk(15, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (45 - 20) + 20))
                        # continue

                else: 
                    myRobot.triangularwalk(myRobot.walkDir.copy())
                # triangularwalk(-20)
                # time.sleep(sleepTime)
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
        myRobot.stopwalknew()
        myRobot.ser.close()
        break

    except serial.SerialTimeoutException:
        myRobot.stopwalknew()
        myRobot.ser.close()
        print("Serial Timeout, exited.")
        break


myRobot.cap.release()
cv2.destroyAllWindows()
exit()
