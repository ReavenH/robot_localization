from zh_Utilities import hmRPYG, robot, ser
import cv2
import time
from final_localization import startwalknew, stopwalknew, freeturn, triangularwalk
import numpy as np

'''
TODO:
1. DONE. Use the Manhattan centroid as the point for distance offset (this ensures that the point is an existing circle).
2. Use larger circles to decide the next action, thus to navigate on a plane.
3. Localize on a 2D structure.
'''

myRobot = robot(hmRPYG, None, None, ser, vidsrc=1)
myRobot.initBottomCamera()

timePrev = time.time()
yawTolerance = 10  # default 5
yawThreshold = 10
isturn = False

sleepTime = 0.5

#startwalknew()
triangularwalk(0)

while myRobot.cap.isOpened:
    try:
        myRobot.readGlobalStep()
        # print("global step:", myRobot.globalStep)
        myRobot.getPoseFromCircles(verbose=False, minCircles=3, display=False)

        if myRobot.globalStep >= 1.0 and myRobot.lostVision == 0:
            centroid = myRobot.bottomLineCentroid
            yaw = - myRobot.bottomLineYaw

            # process yaw to output a correct datatype.
            if len(yaw) == 2:
                yaw = yaw[np.argmin(np.abs(yaw))]
                yaw = np.sign(yaw) * np.abs(yaw)

            if isinstance(yaw, (int, float)) != True:
                yaw = yaw[0]

            print("centroid: [{:.2f}, {:.2f}] | yaw(s): {}\n".format(centroid[0], centroid[1], yaw))

            if np.abs(yaw) > yawThreshold:
                isturn = True

            # control the robot.
            if isturn:
                # stopwalknew()
                if yaw > yawTolerance:
                    freeturn(min(max(yaw*1.5,7.5),15))
                    # time.sleep(sleepTime)
                    continue
                elif yaw < - yawTolerance:
                    freeturn(max(min(yaw*1.5,-7.5),-15))
                    # time.sleep(sleepTime)
                    continue
                else:
                    isturn = False
                    #startwalknew() 
                    continue
            triangularwalk(myRobot.walkDir.copy())
            # triangularwalk(-20)
            # time.sleep(sleepTime)

        elif myRobot.globalStep >= 1.0 and myRobot.lostVision != 0:
            if myRobot.lostVision == 1:
                print("Lost vision on the right.")
                triangularwalk(-50)
            else:
                print("Lost vision on the left.")
                triangularwalk(50)

        else:
            # time.sleep(0.1)
            continue

    except KeyboardInterrupt:
        stopwalknew()
        myRobot.ser.close()
        break

myRobot.cap.release()
cv2.destroyAllWindows()
exit()