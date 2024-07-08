import numpy as np
from zh_Utilities import landmarks, brickMap, hmRPYG, hmRPYP, poseTags, drawGroundOG, drawRigidBodyOG, keyboardCtrl, drawFloor, drawLineOG, robot, R
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import time
from zh_DrawSceneOpenGL import brickPoses, myBrickMap, myTags

'''
TODO:
1. Joint angle rearA-LinkageC is wrong. Done.
2. Coordinate propagation is wrong. (not symmetric). Done.
3. The translation of the linkageA-C joint is wrong (should not be all zeros). Done.
4. Bind the OBJ file to the linkage model (later on).
5. Calculate the body pose based on the standing pose from the ground (if it is needed for verification).
6. Design the brick placing system, first finish the linkage model design.
'''

if __name__ == "__main__":
    
    try:
        myTags = landmarks(hmRPYP, poseTags, None)  # tags use RPYP pose. Not specifying the axes.
        myBrickMap = brickMap(hmRPYG, None)
        myRobot = robot(hmRPYG, None, poseTags)
        myRobot.bodyPose[-1] = myBrickMap.brickThickness + myRobot.initFeetPos[0][1] - myRobot.linkageFrameOffsets[0][-1]
        myRobot.feetPosControl(myRobot.initFeetPos)
        myRobot.propagateAllLegJointPoses()

        pygame.init()  # init the pygame lib.

        window = (1600, 1200)  # in pixels.
        
        pygame.display.set_mode(window, DOUBLEBUF|OPENGL)
        pygame.display.set_caption('DEMO Scene Simulation')

        # enable depth test. This avoids the incorrect transparency between objects.
        # this should be AFTER the pygame / glfw window is initialized.
        glEnable(GL_DEPTH_TEST)

        glClearColor(0.5, 0.5, 0.5, 1.0)

        gluPerspective(60, (window[0] / window[1]), 0.001, 10.0)

        
        glTranslatef(-0.5, -0.5, -1.7)
        glRotatef(-45, 1, 0, 0)  
        glRotatef(45, 0, 0, 1) 
        glTranslatef(0.5, 0, 0)
        
        # get the current view angle.
        hmScene = hmRPYP(-45, 0, 0, np.array([-0.5, -0.5, -1.7])).dot(hmRPYP(0, 0, 45, np.array([0.5, 0, 0])))
        angle_x, angle_y, angle_z = R.from_matrix(hmScene[:3, :3]).as_euler('zyx', degrees=True)[::-1]
        transScene = hmScene[:3, -1].flatten()
        distance = transScene[-1]
        print("angles XYZ: {} {} {}".format(angle_x, angle_y, angle_z))
        print("Translation XYZ: ", transScene)

        while True:
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)  # clear the screen.

            # interactive view angle.
            keyboardCtrl()

            drawFloor()

            for _, pose in enumerate(myTags.poses):
                drawGroundOG(myTags.hm(*pose[1:4], pose[4:]))
                drawRigidBodyOG(myTags.hm(*pose[1:4], pose[4:]).dot(myTags.vertices))

            for _, pose in enumerate(brickPoses):
                 _ = myBrickMap.place(pose)

            # Test the robot class.
            myRobot.drawRobotBody()
            myRobot.drawAllLegLinkagesOG()

            pygame.display.flip()
            pygame.time.wait(10)

    except KeyboardInterrupt:
        pygame.quit()
        quit()