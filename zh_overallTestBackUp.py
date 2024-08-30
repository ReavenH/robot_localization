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
myRobot.switchIMU(False)
myRobot.startwalknew()
myRobot.RPYCtl('yaw', 0)
time.sleep(1)
myRobot.interrupt()
myRobot.triangularwalk(0, distance=0)
myRobot.changeclearance()
myRobot.interrupt()
myRobot.globalStep = 1
myRobot.triangularwalk(0, 20)

lastTurnTime = time.time()

while myRobot.cap.isOpened:
    try:
        myRobot.readGlobalStep()
        # print("global step:", myRobot.globalStep)
        # prevent overcounting in turnings.
        ret = myRobot.getPoseFromCircles(verbose=False, minCircles=5, display=False)
        if ret == None:  # stop the robot if the camera is down.
            myRobot.currentAction = 'S'
        flagHistory = np.append(flagHistory, (len(myRobot.bottomLineYaw) == 2))
        print("isMoving: {} | prevCrossing: {} | atCrossing: {} | currentAction: {} | countCrossing: {} | lostVision: {}".format(myRobot.isMoving, myRobot.prevCrossing, myRobot.atCrossing, myRobot.currentAction, myRobot.countCrossing, myRobot.lostVision))
        
        if myRobot.currentAction == 'S':
            # myRobot.stopwalknew()
            # print("Action Stopped (currentAction is 'S')")
            myRobot.interrupt()
            print("Action Interrputed (currentAction is 'S')")
            break

        elif myRobot.currentAction == 'L':
            myRobot.isCounting = False
            myRobot.interrupt()
            time.sleep(0.7)
            # blind turn, wait until globalStep is greater than 1.0
            myRobot.freeturn(20)
            for i in range(5): # default 5
                # myRobot.waitGlobalStep()
                time.sleep(1.3)
                myRobot.freeturn(20) # default 3zh_planarWalk1.py0
                # time.sleep(2.5)
            myRobot.countCrossing += 1
            myRobot.prevAction = myRobot.currentAction
            myRobot.currentAction = myRobot.path[myRobot.countCrossing]
            lastTurnTime = time.time()

        elif myRobot.currentAction == 'R':
            myRobot.isCounting = False
            myRobot.interrupt()
            time.sleep(0.7)
            # blind turn, wait until globalStep is greater than 1.0
            myRobot.freeturn(-25)
            time.sleep(1.3)
            for i in range(6): # default 5
                # myRobot.waitGlobalStep()
                myRobot.freeturn(-25) # default 3zh_planarWalk1.py0
                time.sleep(1.3)
                for i in range(6):
                    time.sleep(0.1)
                    myRobot.getPoseFromCircles()
                print("LostVision: {}".format(myRobot.lostVision))
                # time.sleep(2.5)
            '''
            for i in range (1):
                myRobot.triangularwalk(0, 35, continuous=False)
                time.sleep(1.2)
            '''
            myRobot.countCrossing += 1
            myRobot.prevAction = myRobot.currentAction
            myRobot.currentAction = myRobot.path[myRobot.countCrossing]
            lastTurnTime = time.time()

        elif myRobot.currentAction == "G":  
            # grab the brick from docker.
            print("Grabing the brick from docker.")
            myRobot.interrupt()
            
            # TODO: use rpyPID control to align the body.
            for i in range(6):
                time.sleep(0.1)
                myRobot.getPoseFromCircles()
                # print("Yaw: {}, lostVision: {}".format(myRobot.bottomLineYawStraight, myRobot.lostVision))
            while abs(myRobot.bottomLineYawStraight) > 8:
                if myRobot.bottomLineYawStraight > 0:
                    myRobot.freeturn(max(min(myRobot.bottomLineYawStraight*1.5,-7.5),-15))
                    time.sleep(1.2)
                elif myRobot.bottomLineYawStraight < 0:
                    myRobot.freeturn(min(max(myRobot.bottomLineYawStraight*1.5,7.5),15))
                    time.sleep(1.2)
                for i in range(6):
                    time.sleep(0.1)
                    myRobot.getPoseFromCircles()
                    # print("Yaw: {}, lostVision: {}".format(myRobot.bottomLineYawStraight, myRobot.lostVision))
            myRobot.RPYCtl('pitch', 0)
            time.sleep(2)
            myRobot.rpyPID(aim=-1, tolerance=1.0)

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
            time.sleep(15)  # adjustable. 27 default.
            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["linkageFlat"], 1/10)
            time.sleep(1)
            myRobot.singleServoCtrl(1, myRobot.servoCriticalAngles["brickFlat"], 1/10)
            time.sleep(1)
            myRobot.closeGripper()
            time.sleep(1)

            # pull out the brick.
            myRobot.currentAction = 'F'
            myRobot.countCrossing += 1
            myRobot.RPYCtl('pitch', 0)
            time.sleep(1)
            myRobot.RPYCtl('yaw', 0)
            time.sleep(1)
            myRobot.interrupt()
            myRobot.triangularwalk(0, distance=myRobot.walkDis, continuous=True)
            myRobot.changeclearance()
            for i in range(4):
                myRobot.waitGlobalStep()
                for j in range(5):
                    time.sleep(0.1)
                    myRobot.getPoseFromCircles()
                myRobot.rlbPID()
                myRobot.rlbControl(myRobot.RLB)
                myRobot.triangularwalk(myRobot.walkDir.copy(), distance=myRobot.walkDis)

            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles['linkageUp'], 1/10)
            
            # notify the docker.
            myRobot.buzzer(True)
            time.sleep(3.5)  # default 4.5
            myRobot.buzzer(False)

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
        elif myRobot.prevAction == 'P' and myRobot.currentAction == 'F':
            myRobot.interrupt()
            myRobot.triangularwalk(0, distance=0)
            myRobot.changeclearance()
            myRobot.interrupt()
            myRobot.triangularwalk(0, 20)

        elif myRobot.currentAction == "P":
            # place the brick.
            myRobot.interrupt()
            # mediate offset.
            if myRobot.prevAction == "R":
                myRobot.triangularwalk(90, 15, continuous=False)
                time.sleep(1.4)
                myRobot.triangularwalk(90, 15, continuous=False)
                time.sleep(1.4)
            # adjust the yaw offset roughly.
            for i in range(7):
                time.sleep(0.1)
                myRobot.getPoseFromCircles()
                print("Yaw: {}, lostVision: {}".format(myRobot.bottomLineYawStraight, myRobot.lostVision))
            while abs(myRobot.bottomLineYawStraight) > 8:
                if myRobot.bottomLineYawStraight > 0:
                    myRobot.freeturn(max(min(myRobot.bottomLineYawStraight*1.5,-7.5),-15))
                    time.sleep(1.2)
                elif myRobot.bottomLineYawStraight < 0:
                    myRobot.freeturn(min(max(myRobot.bottomLineYawStraight*1.5,7.5),15))
                    time.sleep(1.2)
                for i in range(6):
                    time.sleep(0.1)
                    myRobot.getPoseFromCircles()
                    print("Yaw: {}, lostVision: {}".format(myRobot.bottomLineYawStraight, myRobot.lostVision))
            # adjust the lateral offset.
            for i in range(6):
                time.sleep(0.1)
                myRobot.getPoseFromCircles()
                print("Yaw: {}, lostVision: {}".format(myRobot.bottomLineYawStraight, myRobot.lostVision))
            while myRobot.lostVision != 0:
                print("Adjusting lateral offset: lostVision is {}".format(myRobot.lostVision))
                if myRobot.lostVision > 0:
                    if myRobot.lostVision == 2:
                        myRobot.triangularwalk(-90, 10, continuous=False)
                    elif myRobot.lostVision == 1:
                        myRobot.triangularwalk(-90, 12, continuous=False)
                    myRobot.waitGlobalStep()
                elif myRobot.lostVision < 0:
                    if myRobot.lostVision == -2:
                        myRobot.triangularwalk(90, 10, continuous=False)
                    elif myRobot.lostVision == -1:
                        myRobot.triangularwalk(90, 12, continuous=False)
                    myRobot.waitGlobalStep()
                for i in range(6):
                    time.sleep(0.1)
                    myRobot.getPoseFromCircles()
                    print("Yaw: {}, lostVision: {}".format(myRobot.bottomLineYawStraight, myRobot.lostVision))
            # use the RPY PID control from the ESP32 to rotate body without moving feet.
            myRobot.brickAlign()
            time.sleep(1.5)
            myRobot.rpyPID(aim = 0.7)
            myRobot.placeBrickPhase1()
            time.sleep(1)
            myRobot.pushBrick(-35, verbose = True)  # default -32
            time.sleep(4)
            myRobot.rpyPID(aim = 0.7)
            myRobot.pushBrick(35, verbose = True)  # default 32
            time.sleep(4)
            myRobot.placeBrickPhase2()
            myRobot.pushBrick(-30)
            myRobot.placeBrickPhase3()
            myRobot.placeBrickPhase4()
            myRobot.pushBrick(30)
            myRobot.stopwalknew()
            myRobot.changeclearance()  # default 20
            myRobot.RPYCtl('yaw', 0)
            break

        # turn on IMU before climbing.
        elif myRobot.currentAction == "C" and myRobot.isCounting == True:
            # myRobot.stopwalknew()
            myRobot.interrupt()
            time.sleep(1)
            myRobot.walkDis = 40
            myRobot.rlbSetpoint = -10.0  # can be finetuned.
            myRobot.isCounting = False
            # start climbing.
            myRobot.kpPitch(val=0.0125)
            myRobot.startClimbingAPI()
            lastTurnTime = time.time()
            print("Entering C | rlbSetpoint: {}".format(myRobot.rlbSetpoint))

        elif myRobot.prevAction == "F" and myRobot.currentAction == 'F' and myRobot.isClimbing == True:
            # stop climbing API.
            myRobot.rlbSetpoint = 0.0
            myRobot.stopClimbingAPI()
            print("In main C -> F | rlbSetpoint: {}".format(myRobot.rlbSetpoint))

        elif myRobot.globalStep >= 1.0:

            if myRobot.currentAction == 'F' or myRobot.currentAction == 'C':

                print("dt: {}".format(time.time() - lastTurnTime))
                if myRobot.isCounting == False:
                    if myRobot.currentAction == 'F' and (time.time() - lastTurnTime >= 9.0):
                        myRobot.isCounting = True
                
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
                        myRobot.triangularwalk(myRobot.walkDir.copy(), distance=myRobot.walkDis)
                    # tune the params.
                    elif myRobot.lostVision in [-2, 2]:
                        print("into SL")
                        if myRobot.lostVision == 2:
                            print("Slightly lost vision on the right.")
                            myRobot.triangularwalk(-25, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (myRobot.walkDis - 20) + 20))
                            # continue
                        elif myRobot.lostVision == -2:
                            print("Slightly lost vision on the left.")
                            myRobot.triangularwalk(15, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (myRobot.walkDis - 20) + 20))
                            # continue
                    else:
                        # newly added rlbPID during normal walking.
                        myRobot.rlbPID()
                        print("RLB: {}".format(myRobot.RLB))
                        myRobot.rlbControl(myRobot.RLB)
                        myRobot.triangularwalk(myRobot.walkDir.copy(), distance=myRobot.walkDis)

                else:
                    # myRobot.startwalknew()
                    if myRobot.lostVision == 1:
                        print("Lost vision on the right.")
                        if myRobot.currentAction == 'F':
                            myRobot.triangularwalk(-30, myRobot.walkDis)  # default 35
                        elif myRobot.currentAction == 'C':
                            myRobot.triangularwalk(-90, myRobot.walkDis)
                    elif myRobot.lostVision == -1:
                        print("Lost vision on the left.")
                        if myRobot.currentAction == 'F':
                            myRobot.triangularwalk(30, myRobot.walkDis)    
                        elif myRobot.currentAction == 'C':
                            myRobot.triangularwalk(90, myRobot.walkDis)
        # print("-------------------------------------------------------------------")

    except KeyboardInterrupt:
        break

    except serial.SerialTimeoutException:
        print("Serial Timeout, exited...")
        break

    except NameError:
        print("NameError, exited...")
        break

# myRobot.changeclearance()
# myRobot.triangularwalk(0, 0, continuous=False)
myRobot.stopClimbingAPI()

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
