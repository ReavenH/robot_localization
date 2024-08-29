import pickle
import socket
import numpy as np
import sys
import matplotlib.pyplot as plt
from zh_Utilities import landmarks, brickMap, hmRPYG, hmRPYP, poseTags, drawBrick, drawGround, drawRigidBody
import time

# ---- Create a figure object ----
fig0 = plt.figure(0)
ax_fig0 = fig0.add_subplot(111, projection='3d')
ax_fig0.set_xlabel('X')
ax_fig0.set_ylabel('Y')
ax_fig0.set_zlabel('Z')

ax_fig0.set_xlim([-0.25, 1.25])
ax_fig0.set_ylim([-0.25, 1.25])
ax_fig0.set_zlim([0, 1.5])

ax_fig0.grid(False)

myTags = landmarks(hmRPYP, poseTags, ax_fig0)  # tags use RPYP pose.
myBrickMap = brickMap(hmRPYG, ax_fig0)

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

        ax_fig0.view_init(elev = 30, azim = -135)

        for _, pose in enumerate(myTags.poses):
            drawGround(myTags.hm(*pose[1:4], pose[4:]), myTags.ax, "Tag "+str(int(pose[0])))
            drawRigidBody(myTags.hm(*pose[1:4], pose[4:]).dot(myTags.vertices), myTags.ax)

        for _, pose in enumerate(brickPoses):
             _ = myBrickMap.place(pose)
             
        receivedArray = np.zeros(6)

        i = 0
        while(i < pseudoPoses.shape[0]):
            # if use pseudoPoses
            receivedArray = pseudoPoses[i, :]

            # if we use brickPoses
            i += 1

            # plt.plot(trackInner[0, :], trackInner[1, :], trackInner[2, :], color = 'k', linewidth = 2)
            # plt.plot(trackOuter[0, :], trackOuter[1, :], trackOuter[2, :], color = 'k', linewidth = 2)

            
            drawGround(hmRPYG(*receivedArray[:3], receivedArray[3:]), ax_fig0, "")
            drawGround(hmRPYG(0, 0, 0, np.array([0, 0, 0])), ax_fig0, "Home")
            

            plt.pause(0.2)
        
        plt.pause(60)

    except KeyboardInterrupt:
        plt.close()
        
