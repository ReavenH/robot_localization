from zh_Utilities import landmarks, brickMap, hmRPYG, hmRPYP, poseTags, drawGroundOG, drawRigidBodyOG, keyboardCtrl, drawFloor, drawLineOG, robot, R
import cv2
import time

'''
TODO:
1. visual stair detection (for mapping and gait control);
2. determine the correct yaw at the crossing point (when turning);
3. the localization workflow combining both cameras' results (to filter the exact location);
'''

myRobot = robot(hmRPYG, None, poseTags, None, vidsrc=1)
myRobot.initBottomCamera()

timePrev = time.time()
while myRobot.cap.isOpened:
    try:
        '''
        poseBtmCam = myRobot.getPoseFromCircles(minCircles=2, display=False)
        centroid = poseBtmCam[0]
        yaw = poseBtmCam[1]
        '''
        myRobot.getPoseFromCircles(minCircles=3, display=False, verbose=False)
        print("MC: [{:.2f}, {:.2f}] | walkDir: {} | yaw(s): {}\n".format(myRobot.bottomLineCentroid[0], myRobot.bottomLineCentroid[1], myRobot.walkDir, myRobot.bottomLineYaw))
        print("Prev Dots: {}, Next Dots: {}\n".format(myRobot.previousCircles, myRobot.nextCircles))
        print("------------------------------------------------------------------------------")
        time.sleep(0.1)
        '''
        dt = time.time() - timePrev
        fps = 1 / dt
        timePrev = time.time()
        print("FPS: ", fps)
        '''
    except KeyboardInterrupt:
        break

myRobot.cap.release()
cv2.destroyAllWindows()
exit()