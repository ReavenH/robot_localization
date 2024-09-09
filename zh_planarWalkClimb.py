from zh_Utilities import hmRPYG, robot, ser
import cv2
import time
import numpy as np
import os
import serial
from scipy.io import savemat

'''
TODO:
Add RLB compatibility.
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
        # print("-------------------------------------------------------------------")

    except KeyboardInterrupt:
        # myRobot.stopwalknew()
        myRobot.interrupt()
        myRobot.switchIMU(False) 
        myRobot.ser.close()
        break

    except serial.SerialTimeoutException:
        # myRobot.stopwalknew()
        myRobot.interrupt()
        myRobot.switchIMU(False) 
        myRobot.ser.close()
        print("Serial Timeout, exited.")
        break

    except NameError:
        myRobot.interrupt()
        myRobot.switchIMU(False) 
        myRobot.ser.close()
        break

myRobot.cap.release()
cv2.destroyAllWindows()
exit()
