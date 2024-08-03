#!/usr/bin/env python3

import os
os.path.join('/home/pi/.local/lib/python3.11/site-packages')
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import threading
import time
import calibrationfunc as cf
import final_localization as fl
from hp_LineTracking import LineTracking
from zh_Utilities import robot, hmRPYG, ser, UDPSender, trajectory, poseTags, checkTurning, boardBiasesYaw

# ---- Create a figure object ----
fig0 = plt.figure(0)
ax_fig0 = fig0.add_subplot(111, projection='3d')
ax_fig0.set_xlabel('X')
ax_fig0.set_ylabel('Y')
ax_fig0.set_zlabel('Z')

ax_fig0.set_xlim([-0.5, 1.0])
ax_fig0.set_ylim([-0.75, 0.75])
ax_fig0.set_zlim([0, 1.5])

# ---- Instantiate the robot class ----
myRobot = robot(hmRPYG, ax_fig0, poseTags)
poseRecord = np.zeros((1, 6))

# data sender init.
PC_IP = "10.50.9.237"  # should be updated DAILY!!!
PC_Port = 52000
poseSender = UDPSender(PC_IP, PC_Port)

# store the previous pose (control sending speed)
previousPose = np.zeros(6).astype('float')

walkThreadRunning = False

if __name__ == "__main__":

    # import detection module. If this is not within MAIN, this script will not be able to be imported.
    import apriltag_video_picamera4 as avp
    # ---- initiate a apriltag detetcion threading func ----
    def apriltagDetectionThreadFunc():
        avp.apriltag_video(print_log = False, cameraMatrix = cf.cameraMatrix, distCoeffs = cf.distCoeffs, display_stream=False, output_stream = False)

    # ---- Apriltag measurement update ----
    apriltagDetectionThread = threading.Thread(target=apriltagDetectionThreadFunc)
    apriltagDetectionThread.start()
    time.sleep(1)

    # ---- Odometry readings from serial ---- 
    # define a serial read function that returns dyaw and dx.
    # to be wrapped in a thread.
    '''
    odometryReadIsRunning = True  # thread control flag.
    def odometryRead():
        while odometryReadIsRunning:
            myRobot.odometryUpdate(*readIMU())
    odometryReadThread = threading.Thread(target=odometryRead)
    odometryReadThread.start()
    '''

    # ---- Initialize the LineTracking Class ----
    lineTracking = LineTracking()

    try:  
        # myRobot.forward()

        while(1):
            # ---- Update Odometry ----
            # myRobot.odometryUpdate(*readIMU())

            # ---- Line Recognition ----
            
            frameRead = avp.frameGlobal.copy()
            yOffsetLine = lineTracking.run(frameRead.copy())  # horizontal offset from the center line.
            # print("Time: {} \t yOffsetLine: {} \t No. Frame: {} \t Shape: {} \t Var of Frame: {} \t Type: {} \t".format(time.time(), yOffsetLine, avp.frameCountGlobal, avp.frameGlobal.shape, np.var(avp.frameGlobal[:, :, ::-1]), frameRead.dtype))
            # print("yOffsetLine: {}".format(yOffsetLine))
            
            # plt.imsave("./TESTIMGS/IMG"+str(avp.frameCountGlobal)+".png", avp.frameGlobal[:, :, ::-1])
            # plt.imshow(avp.frameGlobal[:, :, ::-1])
           
            if avp.resultsGlobal != []:

                # ---- Update Measurement ----
                # TODO: only update measurement when the avp.resultsGlobal is different from the previous one.

                myRobot.measurementUpdate(avp.resultsGlobal)

                if not np.all(myRobot.measurement.copy() == previousPose.copy()):  
                    # ---- Print Debug Info ----
                    # drawGround(hmRPYG(*myRobot.measurement[:3], myRobot.measurement[3:]), ax_fig0, "")
                    # print("Measurement at {} : {}".format(time.time(), myRobot.measurement))

                    # ---- Send Data via WiFi ----
                    poseSender.send(np.append(time.time(), myRobot.measurement.copy()))  # send the pose array.

                    # ---- Calculate Params for Walking ---- 
                    previousPose = myRobot.measurement.copy()  # store the previous pose.
                    yawInTag, targetVector = checkTurning(myRobot.trajectoryNo, myRobot.measurement, trajectory)
                    targetVector *= 100  # transform to cm

                    # ---- Pass Params to final_localization.py ----
                    yawInTag += boardBiasesYaw[myRobot.trajectoryNo]
                    turningPose = np.append(yawInTag, targetVector.flatten())
                    turningPose[-1] = yOffsetLine  # use line detection offset
                    # print(turningPose)
                    fl.robotPose = turningPose  # pass the pose to the thread.
                    # print(yawInTag, targetVector)
                    # np.savetxt('robotPose.csv', turningPose, delimiter=',')
                    # print(turningPose)
                    time.sleep(0.1)
                    
                    if not walkThreadRunning:
                        # from Yao's code.
                        # fl.reset()
                        
                        fl.walkThread.start()
                        print('walk Thread start')
                        time.sleep(2)
                        walkThreadRunning = True
                    
            time.sleep(0.1)


    except KeyboardInterrupt:
        
        # myRobot.stopFB()

        # ---- Kill the Apriltag threads. ----
        avp.aptIsRunning = False
        apriltagDetectionThread.join()

        # ---- Kill the odometry update threads. ---- 
        # odometryReadIsRunning = False
        # odometryReadThread.join()

        # ---- control the walking thread ----
        
        walkThreadRunning = False
        fl.walkThread.join()
        fl.iswalk=0
        print('STOOOOOOOOOOOOOOOOOOOOOP')
        

        # ---- Close the WiFi data sender ----
        poseSender.close()
        
        print("exited main")
        plt.close()
        ser.close()
