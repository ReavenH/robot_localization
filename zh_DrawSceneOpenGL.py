import pickle
import socket
import numpy as np
import sys
from zh_Utilities import landmarks, brickMap, hmRPYG, hmRPYP, poseTags, drawGroundOG, drawRigidBodyOG, keyboardCtrl, drawFloor
import time
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

myTags = landmarks(hmRPYP, poseTags, None)  # tags use RPYP pose. Not specifying the axes.
myBrickMap = brickMap(hmRPYG, None)

# create a pseudoPose series
pseudoPoses = np.zeros((10, 6))
pseudoPoses[:, :3] += np.random.normal(0, 5, size=(pseudoPoses.shape[0], 3))
pseudoPoses[:, 3] = np.linspace(0, 0.8, pseudoPoses.shape[0])
pseudoPoses[:, 3:] += np.random.normal(0, 0.01, size=(pseudoPoses.shape[0], 3))
pseudoPoses[:, -1] = myBrickMap.brickThickness * 2.5 * myBrickMap.brickThickness
# print(pseudoPoses)

# create a map of bricks of the DEMO circle.
brickPoses = np.array([[0, 0, 0, 0.3, 0, myBrickMap.brickThickness / 2],
                       [0, 0, 0, 0.7, 0, myBrickMap.brickThickness / 2],
                       [0, 0, 90, 0.8, 0.3, myBrickMap.brickThickness / 2], 
                       [0, 0, 90, 0.8, 0.7, myBrickMap.brickThickness / 2],
                       [0, 0, 180, 0.5, 0.8, myBrickMap.brickThickness / 2], 
                       [0, 0, 180, 0.1, 0.8, myBrickMap.brickThickness / 2], 
                       [0, 0, -90, 0, 0.5, myBrickMap.brickThickness / 2],
                       [0, 0, -90, 0, 0.1, myBrickMap.brickThickness / 2],
                       [0, 0, 0, 0.5, 0, 1.5 * myBrickMap.brickThickness],
                       [0, 0, 90, 0.8, 0.5, 1.5 * myBrickMap.brickThickness],
                       [0, 0, 180, 0.3, 0.8, 1.5 * myBrickMap.brickThickness],
                       [0, 0, -90, 0, 0.3, 1.5 * myBrickMap.brickThickness]])

if __name__ == "__main__":

    try:

        pygame.init()  # init the pygame lib.

        window = (1200, 900)  # in pixels.

        pygame.display.set_mode(window, DOUBLEBUF|OPENGL)
        pygame.display.set_caption('DEMO Scene Simulation')

        glClearColor(0.5, 0.5, 0.5, 1.0)

        gluPerspective(60, (window[0] / window[1]), 0.001, 10.0)

        # glTranslatef(0, 0, -2)
        glTranslatef(-0.5, -0.5, -1.7)
        glRotatef(-45, 1, 0, 0)  
        glRotatef(45, 0, 0, 1) 
        glTranslatef(0.5, 0, 0)
        

        receivedArray = np.zeros(6)

        i = 0
        while(True):

            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)  # clear the screen.

            keyboardCtrl()

            drawFloor()

            for _, pose in enumerate(myTags.poses):
                drawGroundOG(myTags.hm(*pose[1:4], pose[4:]))
                drawRigidBodyOG(myTags.hm(*pose[1:4], pose[4:]).dot(myTags.vertices))

            for _, pose in enumerate(brickPoses):
                 _ = myBrickMap.place(pose)
             

            # if use pseudoPoses
            if i < pseudoPoses.shape[0]:
                receivedArray = pseudoPoses[i, :]

                # if we use brickPoses
                i += 1

                drawGroundOG(hmRPYG(*receivedArray[:3], receivedArray[3:]))
                drawGroundOG(hmRPYG(0, 0, 0, np.array([0, 0, 0])))  # home
            
            pygame.display.flip()
            pygame.time.wait(10)

    except KeyboardInterrupt:
        pygame.quit()
        quit()
        
