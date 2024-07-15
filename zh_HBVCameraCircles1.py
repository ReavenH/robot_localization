from zh_Utilities import landmarks, brickMap, hmRPYG, hmRPYP, poseTags, drawGroundOG, drawRigidBodyOG, keyboardCtrl, drawFloor, drawLineOG, robot, R
import cv2
import time

myRobot = robot(hmRPYG, None, poseTags, None, vidsrc=1)
myRobot.initBottomCamera()

timePrev = time.time()
while myRobot.cap.isOpened:
    try:
        poseBtmCam = myRobot.getPoseFromCircles(verbose=False, minCircles=2, display=False)
        centroid = poseBtmCam[0]
        yaw = poseBtmCam[1]
        print("centroid: [{:.2f}, {:.2f}] | yaw(s): {}\n".format(centroid[0], centroid[1], yaw))
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