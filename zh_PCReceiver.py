import pickle
import socket
import numpy as np
import sys
import matplotlib.pyplot as plt
# from zh_robotPose import landmarks, poseTags, hmRPYG, hmRPYP, drawGround, drawRigidBody, robot
from zh_Utilities import *
import time
import threading

class PCReceiver():
    def __init__(self, local_port):
        self.local_port = local_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.local_port))  # to be decided.

    def receive_array(self):
        data, addr = self.sock.recvfrom(256)  # the message of 6 DoF pose takes up 152 Bytes.
        array = pickle.loads(data)  # deserialize message
        return array, addr

    def close(self):
        self.sock.close()

if __name__ == "__main__":

    RPi_Port = 52000
    receiver = PCReceiver(RPi_Port)

    # ---- Create a figure object ----
    fig0 = plt.figure(0)
    ax_fig0 = fig0.add_subplot(111, projection='3d')
    ax_fig0.set_xlabel('X')
    ax_fig0.set_ylabel('Y')
    ax_fig0.set_zlabel('Z')
    
    ax_fig0.set_xlim([-0.5, 1.0])
    ax_fig0.set_ylim([-0.75, 0.75])
    ax_fig0.set_zlim([0, 1.5])

    myTags = landmarks(hmRPYP, poseTags, ax_fig0)  # tags use RPYP pose.

    for _, pose in enumerate(myTags.poses):
            drawGround(myTags.hm(*pose[1:4], pose[4:]), myTags.ax, "Tag "+str(int(pose[0])))
            drawRigidBody(myTags.hm(*pose[1:4], pose[4:]).dot(myTags.vertices), myTags.ax)

    receivedArray = np.zeros((1, 6))

    # define a thread to plot.
    def plotThreadFunc():
        while(True):
            drawGround(hmRPYG(*receivedArray[:3], receivedArray[3:]), ax_fig0, "")
            time.sleep(1)

    # init the threading function.
    plotThread = threading.Thread(target=plotThreadFunc)
    plotThread.start()

    try:
        while(True):
            receivedArray, _ = receiver.receive_array()
            # drawGround(hmRPYG(*array[:3], array[3:]), ax_fig0, "")
            print("Received Array: {}, Is Numpy Array: {}, Size of Message: {} Bytes".format(receivedArray, isinstance(receivedArray, np.ndarray), sys.getsizeof(receivedArray)))

    except KeyboardInterrupt:
        receiver.close()
        plotThread.join()
        plt.close()
