from zh_Utilities import hmRPYG, robot, ser
import cv2
import time
import numpy as np
import os
import serial
from scipy.io import savemat
from collections import deque
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
myRobot.switchIMU(False)

lastTurnTime = time.time()
myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["linkageUp"], 1/2)
while myRobot.cap.isOpened:
    try:
        print("isMoving: {} | isClimbing {}, {} | prevCrossing: {} | atCrossing: {} | pprevCrossing: {} | prevAction: {} | currentAction: {} | countCrossing: {} | lostVision: {}".format(myRobot.isMoving, myRobot.isClimbing, myRobot.isClimbing1, myRobot.prevCrossing, myRobot.atCrossing, myRobot.pprevAction, myRobot.prevAction, myRobot.currentAction, myRobot.countCrossing, myRobot.lostVision))
        # print("isMoving: {} | currentAction: {} | countCrossing: {} | lostVision: {}".format(myRobot.isMoving, myRobot.currentAction, myRobot.countCrossing, myRobot.lostVision))
        if myRobot.currentAction == 'S':
            # myRobot.stopwalknew()
            # print("Action Stopped (currentAction is 'S')")
            myRobot.interrupt()
            print("Action Interrputed (currentAction is 'S')")
            break

        elif myRobot.currentAction == "Q":
            # align the brick.
            myRobot.stopwalknew()
            time.sleep(0.5)
            myRobot.isCounting = False
            myRobot.buzzer(True)
            myRobot.brickAlign()
            time.sleep(1.2)  # default 4.5
            myRobot.buzzer(False)
            myRobot.singleServoCtrl(1, myRobot.servoCriticalAngles['brickUp'], 1/10)
            time.sleep(0.5)
            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles['linkageUp'], 1/4)
            time.sleep(0.5)
            myRobot.updateActionHistory()
            lastTurnTime = time.time()
            myRobot.globalStep = 1.0

        elif myRobot.currentAction == 'L':
            myRobot.isCounting = False
            myRobot.resetFIFO()
            myRobot.interrupt()
            time.sleep(0.7)
            # mediate bias.
            myRobot.triangularwalk(90, 12, continuous = False)
            # blind turn, wait until globalStep is greater than 1.0
            myRobot.freeturn(25)  # default 20
            for i in range(7): # default 5
                # myRobot.waitGlobalStep()
                time.sleep(1.7)
                myRobot.freeturn(25) # default zh_planarWalk1.py0
                # time.sleep(2.5)
            myRobot.updateActionHistory()
            myRobot.globalStep = 1.0
            myRobot.atCrossing = False
            myRobot.prevCrossing = False
            lastTurnTime = time.time()
            myRobot.interrupt()

        elif myRobot.currentAction == 'K':
            # left turn on brick.
            myRobot.isCounting = False
            myRobot.resetFIFO()
            myRobot.interrupt()
            time.sleep(0.7)
            # mediate bias.
            myRobot.triangularwalk(90, 12, continuous = False)
            # blind turn, wait until globalStep is greater than 1.0
            myRobot.freeturn(25)  # default 20
            for i in range(3): # default 5
                # myRobot.waitGlobalStep()
                time.sleep(1.7)
                myRobot.freeturn(25) # default zh_planarWalk1.py0
                # time.sleep(2.5)
            myRobot.triangularwalk(0, 35, continuous = False)
            for i in range(4): # default 5
                # myRobot.waitGlobalStep()
                time.sleep(1.7)
                myRobot.freeturn(25) # default zh_planarWalk1.py0
                # time.sleep(2.5)
            myRobot.updateActionHistory()
            myRobot.globalStep = 1.0
            myRobot.atCrossing = False
            myRobot.prevCrossing = False
            lastTurnTime = time.time()
            myRobot.interrupt()

        elif myRobot.currentAction == 'R':
            myRobot.isCounting = False
            myRobot.resetFIFO()
            myRobot.interrupt()
            time.sleep(0.7)
            # blind turn, wait until globalStep is greater than 1.0
            myRobot.freeturn(-25)
            time.sleep(1.3)
            for i in range(7): # default 5
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
            myRobot.walkWithVision(steps = 2)
            myRobot.updateActionHistory()
            myRobot.globalStep = 1.0
            lastTurnTime = time.time()
            myRobot.interrupt()
            myRobot.switchIMU(False)

        elif myRobot.currentAction == "G":  
            # grab the brick from docker.
            print("Grabing the brick from docker.")
            myRobot.interrupt()
            myRobot.isCounting = False
            myRobot.stopClimbingAPI()
            myRobot.resetFIFO()
            # walk with vision.
            myRobot.walkWithVision()
            myRobot.adjustLateralOffsetLostVision()
            myRobot.adjustYawByFreeturn(8)  # use freeturn first.
            # adjust lateral offset.
            # myRobot.adjustLateralOffset(10)      
            # Use rpyPID control to align the body.
            # myRobot.adjustYawByFreeturn(8)  # use freeturn first.
            # myRobot.adjustLateralOffsetLostVision()
            myRobot.walkWithVision(steps = 1)
            # myRobot.adjustLateralOffset(7) 
            myRobot.adjustLateralOffsetLostVision()
            myRobot.adjustYawByFreeturn(6) 
            myRobot.RPYCtl('pitch', 0)
            time.sleep(2)
            myRobot.rpyPID(aim=-1, tolerance=1.0)

            while True:
                keyInput = input("Press O to finish loading pins...")
                if keyInput.lower() == 'o':
                    break

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
            '''
            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["linkageDown"], 1/10)
            time.sleep(1)
            '''
            myRobot.closeGripper()
            time.sleep(0.1)
            myRobot.openGripper()
            time.sleep(0.1)
            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["linkageGuideBrick"], 1/10)
            time.sleep(21.15)  # adjustable. 27 default.
            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles["linkageFlat"], 1/10)
            time.sleep(1)
            myRobot.singleServoCtrl(1, myRobot.servoCriticalAngles["brickFlat"], 1/10)
            time.sleep(1)
            myRobot.closeGripper()
            time.sleep(1)
            while True:
                keyInput = input("Press K to finish measuring...")
                if keyInput.lower() == 'k':
                    break

            # pull out the brick.
            myRobot.updateActionHistory()
            myRobot.RPYCtl('pitch', 0)
            time.sleep(1)
            myRobot.RPYCtl('yaw', 0)
            time.sleep(1)
            myRobot.withBrick(1)
            myRobot.interrupt()
            myRobot.stopClimbingAPI()
            myRobot.triangularwalk(0, distance=myRobot.walkDis, continuous=True)
            myRobot.changeclearance()
            for i in range(4):
                myRobot.waitGlobalStep()
                for j in range(5):
                    time.sleep(0.1)
                    myRobot.getPoseFromCircles()
                myRobot.rlbPID()
                myRobot.rlbControl(myRobot.RLB)
                myRobot.triangularwalk(myRobot.walkDir.copy(), distance=myRobot.walkDis - 3)

            myRobot.singleServoCtrl(0, myRobot.servoCriticalAngles['linkageUp'], 1/10)
            myRobot.resetFIFO()
            # notify the docker.
            myRobot.stopwalknew()
            # myRobot.buzzer(True)
            # myRobot.brickAlign()
            # time.sleep(1.2)  # default 4.5
            # myRobot.buzzer(False)
            # lastTurnTime = time.time()

        elif myRobot.currentAction == "P":
            myRobot.setTargetPitch(dval = 0)
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
            '''
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
            '''
            for i in range(6):
                time.sleep(0.2)
                myRobot.getPoseFromCircles()
                print("Yaw: {}, lostVision: {}, bottomLineCentroidCrossing: {}".format(myRobot.bottomLineYawStraight, myRobot.lostVision, myRobot.bottomLineCentroidCrossing))
            while abs(myRobot.bottomLineCentroidCrossing[0]) > 20:
                print("Adjusting lateral offset: centroid x is {}".format(myRobot.bottomLineCentroidCrossing[0]))
                if myRobot.bottomLineCentroidCrossing[0] > 0:
                    myRobot.triangularwalk(-90, np.ceil(np.abs(myRobot.bottomLineCentroidCrossing[0]) / 80 * (18 - 12) + 12), continuous = False)
                    myRobot.waitGlobalStep()
                elif myRobot.bottomLineCentroidCrossing[0] < 0:
                    myRobot.triangularwalk(90, np.ceil(np.abs(myRobot.bottomLineCentroidCrossing[0]) / 80 * (18 - 12) + 12), continuous = False)
                    myRobot.waitGlobalStep()
                time.sleep(0.1)
                centroidXAvg = 0.0
                for i in range(6):
                    time.sleep(0.1)
                    myRobot.getPoseFromCircles()
                    if i >= 4:
                        centroidXAvg += myRobot.bottomLineCentroidCrossing[0]
                        centroidXAvg *= 0.5
                    print("Yaw: {}, lostVision: {}, centroid-x {}, Avg X {}".format(myRobot.bottomLineYawStraight, myRobot.lostVision, myRobot.bottomLineCentroidCrossing[0], centroidXAvg))
            # use the RPY PID control from the ESP32 to rotate body without moving feet.
            myRobot.brickAlign()
            time.sleep(1.5)
            myRobot.rpyPID(aim = -2)
            myRobot.placeBrickPhase1()
            time.sleep(1)
            if myRobot.countPlaced in [2]:
                myRobot.push = False
            else:
                myRobot.push = True
            if myRobot.push:
                myRobot.pushBrick(-35, verbose = True)  # default -32
                time.sleep(4)
            else:
                # myRobot.pushBrick(-25, verbose = True)
                myRobot.fbPID()
            myRobot.rpyPID(aim = -2)
            if myRobot.push:
                myRobot.pushBrick(40, verbose = True)  # default 32
                time.sleep(4)
            myRobot.placeBrickPhase2()
            if myRobot.push:
                myRobot.pushBrick(-35)
            else:
                myRobot.pushBrick(-15)
                myRobot.accumulatedSwing += (-15)
            myRobot.placeBrickPhase3()
            myRobot.placeBrickPhase4()
            if myRobot.push:
                myRobot.pushBrick(30)
            else:
                # myRobot.pushBrick(25, verbose = True)
                myRobot.pushBrick(-myRobot.accumulatedSwing, verbose = True)
                myRobot.accumulatedSwing = 0.0
            myRobot.withBrick(0)
            myRobot.stopwalknew()
            myRobot.changeclearance()  # default 20
            myRobot.RPYCtl('yaw', 0)
            # change back the walking mode.
            myRobot.triangularwalk(0, distance=0, waitAck = False)
            myRobot.changeclearance()
            myRobot.interrupt()
            myRobot.globalStep = 1.0
            # to the next movement.
            myRobot.updateActionHistory()
            myRobot.countPlaced += 1

        #sortclimb
        elif myRobot.currentAction == "C" and myRobot.isClimbing == False:  # default: isCounting == True.
            myRobot.isCounting = False
            myRobot.isClimbing = True
            myRobot.resetFIFO()
            myRobot.switchIMU(False)
            # myRobot.stopwalknew()
            myRobot.interrupt()
            myRobot.climbDetectedThreshold(val = 10)
            time.sleep(0.5)
            # rlb walk forward 1 step.
            for i in range(4):
                myRobot.getPoseFromCircles()
                time.sleep(0.1)
            myRobot.rlbPID()    #mapping yaw to rlb value
            myRobot.rlbControl(val=myRobot.RLB)     #send rlb value to arduino
            myRobot.triangularwalk(myRobot.walkDir, myRobot.walkDis, continuous = False)
            myRobot.interrupt() #?
            time.sleep(0.5)
            # walk at low clearance.
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
            for i in range(6):
                time.sleep(0.1)
                myRobot.getPoseFromCircles()
            while abs(myRobot.bottomLineYawStraight) > 4:
                if myRobot.bottomLineYawStraight > 0:
                    myRobot.freeturn(max(min(myRobot.bottomLineYawStraight*1.5,-12.5),-20))
                    time.sleep(1.2)
                elif myRobot.bottomLineYawStraight < 0:
                    myRobot.freeturn(min(max(myRobot.bottomLineYawStraight*1.5,12.5),20))
                    time.sleep(1.2)
                for i in range(6):
                    time.sleep(0.1)
                    myRobot.getPoseFromCircles()

            myRobot.resetIMU()

            #walk in low clearance
            myRobot.changeclearance(val=10)
            for i in range(3):
                for j in range(8):
                    time.sleep(0.1)
                    myRobot.getPoseFromCircles()
                myRobot.rlbPID()
                myRobot.rlbControl(val=myRobot.RLB)
                myRobot.triangularwalk(myRobot.walkDir, distance = myRobot.walkDis, continuous = False)
                myRobot.waitGlobalStep()
            
            #turn to adjust yaw
            time.sleep(1)
            myRobot.freeturn(-20)  #initially -25

            myRobot.waitGlobalStep()
            myRobot.walkDis = 50  # default 45.
            # myRobot.rlbSetpoint = -10.0  # can be finetuned.
            myRobot.executeRLB = False      #mannually change RLB in this state 
            myRobot.RLB = 0.0  # default -100
            # start climbing.
            myRobot.kpPitch(val=0.006)
            if myRobot.countCrossing == 0:
                myRobot.discrete_startClimbingAPI()
            myRobot.discrete_startClimbingAPI()
            myRobot.startClimbingAPI()
            # lastTurnTime = time.time()
            print("Entering C | rlbSetpoint: {}".format(myRobot.rlbSetpoint))

        elif myRobot.currentAction == "A":
            '''
            The first two.
            '''
            # walk for 1 cycle.
            myRobot.stopClimbingAPI()
            for j in range(2):
                for i in range(6):
                    time.sleep(0.1)
                    myRobot.getPoseFromCircles()
                myRobot.rlbPID()
                myRobot.rlbControl(myRobot.RLB)
                myRobot.triangularwalk(myRobot.walkDir, myRobot.walkDis - 5, continuous = False)
                myRobot.waitGlobalStep()
            myRobot.stopwalknew()
            # firstly adjust the lateral offset.
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
            for i in range(6):
                time.sleep(0.1)
                myRobot.getPoseFromCircles()
            while abs(myRobot.bottomLineYawStraight) > 4:
                if myRobot.bottomLineYawStraight > 0:
                    myRobot.freeturn(max(min(myRobot.bottomLineYawStraight*1.5,-12.5),-20))
                    time.sleep(1.2)
                elif myRobot.bottomLineYawStraight < 0:
                    myRobot.freeturn(min(max(myRobot.bottomLineYawStraight*1.5,12.5),20))
                    time.sleep(1.2)
                for i in range(6):
                    time.sleep(0.1)
                    myRobot.getPoseFromCircles()
            # pin down the first two nails.
            myRobot.two_nails()
            # change back the walking mode.
            myRobot.triangularwalk(0, distance=0, waitAck = False)
            myRobot.changeclearance()
            myRobot.interrupt()
            myRobot.globalStep = 1.0
            # to the next movement.
            myRobot.updateActionHistory()
            # myRobot.triangularwalk(0, myRobot.walkDis)

        elif myRobot.pprevAction == "C" and myRobot.prevAction == "F" and myRobot.currentAction != "C" and myRobot.isClimbing1 == False:
            myRobot.stopClimbingAPI()
            myRobot.isClimbing1 = True
            time.sleep(0.2)
            # myRobot.resetIMU()
            myRobot.adjustedPose = False

        elif myRobot.pprevAction == "C" and myRobot.prevAction == "F" and myRobot.adjustedPose == False:
            # adjust its lateral offset.
            myRobot.stopwalknew()
            myRobot.triangularwalk(0, 40, continuous = False)
            myRobot.waitGlobalStep
            for i in range(8):
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
            # then adjust the yaw offset.
            myRobot.interrupt()
            for i in range(8):
                time.sleep(0.1)
                myRobot.getPoseFromCircles()
            while abs(myRobot.bottomLineYawStraight) > 3:
                if myRobot.bottomLineYawStraight > 0:
                    myRobot.freeturn(max(min(myRobot.bottomLineYawStraight*1.5,-12.5),-20))
                    time.sleep(1.2)
                elif myRobot.bottomLineYawStraight < 0:
                    myRobot.freeturn(min(max(myRobot.bottomLineYawStraight*1.5,12.5),20))
                    time.sleep(1.2)
                for i in range(8):
                    time.sleep(0.1)
                    myRobot.getPoseFromCircles()
            myRobot.adjustedPose = True
            myRobot.globalStep = 1.0
            # myRobot.isCounting = False
            # lastTurnTime = time.time() + 5
            myRobot.resetFIFO()

        elif myRobot.currentAction == "V":
            '''
            The latter two.
            '''
            # TODO: walk for another 8 cycles.
            myRobot.stopwalknew()

            for j in range(2):
                for i in range(6):
                    time.sleep(0.1)
                    myRobot.getPoseFromCircles()
                myRobot.rlbPID()
                myRobot.rlbControl(myRobot.RLB * 1.01)
                myRobot.triangularwalk(myRobot.walkDir, myRobot.walkDis, continuous = False)
                myRobot.waitGlobalStep()
            
            # firstly adjust the lateral offset.
            myRobot.stopwalknew()
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
            # then adjust the yaw offset.
            myRobot.interrupt()
            for i in range(8):
                time.sleep(0.1)
                myRobot.getPoseFromCircles()
            while abs(myRobot.bottomLineYawStraight) > 3:
                if myRobot.bottomLineYawStraight > 0:
                    myRobot.freeturn(max(min(myRobot.bottomLineYawStraight*1.5,-12.5),-20))
                    time.sleep(1.2)
                elif myRobot.bottomLineYawStraight < 0:
                    myRobot.freeturn(min(max(myRobot.bottomLineYawStraight*1.5,12.5),20))
                    time.sleep(1.2)
                for i in range(8):
                    time.sleep(0.1)
                    myRobot.getPoseFromCircles()
            # myRobot.interrupt()

            # pin down the latter two nails.
            myRobot.two_nails_on_board()
            # change back the walking mode.
            myRobot.triangularwalk(0, distance=0, waitAck = False)
            myRobot.changeclearance()
            myRobot.interrupt()
            myRobot.globalStep = 1.0
            # to the next movement.
            myRobot.updateActionHistory()
            # myRobot.triangularwalk(0, myRobot.walkDis)

        elif myRobot.prevAction == "C" and myRobot.currentAction == 'F' and myRobot.isClimbing == True:
            # 2nd stage of climbing.
            # stop climbing API.
            myRobot.rlbSetpoint = 0.0
            # myRobot.stopClimbingAPI()
            # myRobot.setTargetPitch(dval = 15)
            myRobot.switchIMU(False)
            myRobot.isClimbing = False
            myRobot.isClimbing1 = False
            print("In main, exiting C | rlbSetpoint: {}".format(myRobot.rlbSetpoint))

        elif myRobot.currentAction == 'F' or myRobot.currentAction == 'C':
        
            if myRobot.isMoving:
                myRobot.readGlobalStep()

            ret = myRobot.getPoseFromCircles(verbose=False, minCircles=5, display=False)
            if ret == None:  # stop the robot if the camera is down.
                myRobot.currentAction = 'S'
                
            
            if myRobot.globalStep >= 1.0:
                print("dt: {}".format(time.time() - lastTurnTime))

                if myRobot.isCounting == False:
                    if myRobot.currentAction == 'F' and (time.time() - lastTurnTime >= 7.0):
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
                        myRobot.triangularwalk(myRobot.walkDir.copy(), distance=myRobot.walkDis, waitAck = (myRobot.prevAction == 'F'))
                    # tune the params.
                    elif myRobot.lostVision in [-2, 2]:
                        print("into SL")
                        if myRobot.lostVision == 2:
                            print("Slightly lost vision on the right.")
                            if myRobot.currentAction == 'F':
                                myRobot.triangularwalk(-30, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (myRobot.walkDis - 23) + 23), waitAck = (myRobot.prevAction == 'F'))
                            elif myRobot.currentAction == 'C':
                                myRobot.triangularwalk(myRobot.walkDir, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (myRobot.walkDis - 23) + 23), waitAck = (myRobot.prevAction == 'F'))
                            # continue
                        elif myRobot.lostVision == -2:
                            print("Slightly lost vision on the left.")
                            if myRobot.currentAction == 'F':    
                                myRobot.triangularwalk(15, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (myRobot.walkDis - 20) + 20), waitAck = (myRobot.prevAction == 'F'))
                            elif myRobot.currentAction == 'C':
                                myRobot.triangularwalk(myRobot.walkDir, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (myRobot.walkDis - 23) + 23), waitAck = (myRobot.prevAction == 'F'))
                            # continue
                    else:
                        # newly added rlbPID during normal walking.
                        myRobot.rlbPID()
                        print("RLB: {}".format(myRobot.RLB))
                        myRobot.rlbControl(myRobot.RLB)
                        myRobot.triangularwalk(myRobot.walkDir.copy(), distance=myRobot.walkDis, waitAck = (myRobot.prevAction == 'F'))
                else:
                    # myRobot.startwalknew()
                    if myRobot.lostVision == 1:
                        print("Lost vision on the right.")
                        if myRobot.currentAction == 'F':
                            myRobot.triangularwalk(-30, myRobot.walkDis, waitAck = (myRobot.prevAction == 'F'))  # default 35
                        elif myRobot.currentAction == 'C':
                            myRobot.triangularwalk(-90, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (30 - 12) + 12), waitAck = (myRobot.prevAction == 'F'))
                    elif myRobot.lostVision == -1:
                        print("Lost vision on the left.")
                        if myRobot.currentAction == 'F':
                            myRobot.triangularwalk(30, myRobot.walkDis, waitAck = (myRobot.prevAction == 'F'))    
                        elif myRobot.currentAction == 'C':
                            myRobot.triangularwalk(90, np.ceil(np.abs(myRobot.bottomLineCentroid[1]) / 120 * (30 - 12) + 12), waitAck = (myRobot.prevAction == 'F'))

    except KeyboardInterrupt:
        break

    except serial.SerialTimeoutException:
        print("Serial Timeout, exited...")
        break
    '''
    except NameError:
        print("NameError, exited...")
        break
    '''

# myRobot.changeclearance()
# myRobot.triangularwalk(0, 0, continuous=False)
myRobot.stopClimbingAPI()
myRobot.setTargetPitch(dval = 0)
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
