#!/usr/bin/env python3

import os
os.path.join('/home/pi/.local/lib/python3.11/site-packages')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading
import time
import calibrationfunc as cf
import serial
import json
import re
import socket
import pickle
import final_localization as fl

# from WAVESHARE, establish serial connection.
def connect_serial(port, baudrate):
    while True:
        try:
            ser = serial.Serial(port, baudrate)
            print("Serial connected")
            return ser
        except serial.SerialException as e:
            print("Serial Disconnected:", e)
            print("wait for 5 second for reconnecting...")
            time.sleep(5)

##kill the agetty serial
os.system("sudo systemctl stop serial-getty@ttyS0.service")
os.system("sudo chmod 777 /dev/ttyS0")

port = "/dev/ttyS0"
baudrate = 115200
ser = connect_serial(port, baudrate)

def readIMU():
    
    # decode each line in the buffer.
    try:
        value_str = ser.readline().decode().strip()
    except Exception as e:
        print(f"Error reading from serial port: {e}")
        return None, None, None
 
    robot_time = re.search(r'Time: (\d+)', value_str)  # int
    yaw = re.search(r'Yaw: (-?\d+\.\d+)', value_str)  # float
    dx = re.search(r'dx: (-?(\d+\.\d+|inf))', value_str)  # float

    if robot_time:
        # get the time in second
        stamp = int(robot_time.group(1))
    else:
        stamp = None
        
    if yaw:
        yaw_out = yaw.group(1)
    else:
        yaw_out = None
        
    if dx:
        dx_out = dx.group(1)
    else:
        dx_out = None

    print(stamp, yaw_out, dx_out)
    return stamp, yaw_out, dx_out


# Calibrate the Yaw and X, Z translation (based on Yao and Peng's code)
def calibratePose2D(yaw, trans):
    # Calibrate Yaw:
    biasAngle = np.arcsin(trans[0] / trans[-1])
    biasAngle = np.degrees(biasAngle)
    yaw = yaw + biasAngle
    yawRad = np.radians(yaw)

    # calibrate X translation.
    trans[0] = trans[-1] * np.sin(yawRad)
    trans[-1] = trans[-1] * np.cos(yawRad)

    return trans

# define the ground coordinates:
# X: pointing forwards; Y: pointing left; Z: pointing upwards
# each column: x, y, z point coords,
groundCoords = np.array([[1.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0],
                         [0.0, 0.0, 1.0]])


def rotX(phi):
    '''
    rotation matrix around the X axis (roll)
    np.cos, np.sin uses rads as input
    '''
    phi = np.deg2rad(phi)
    rotation_matrix_X = np.array([[1.0, 0.0, 0.0],
                                  [0.0, np.cos(phi), - np.sin(phi)],
                                  [0.0, np.sin(phi), np.cos(phi)]])
    return rotation_matrix_X


def rotY(theta):
    '''
    rotation matrix around the Y axis (pitch)
    '''
    theta = np.deg2rad(theta)
    rotation_matrix_Y = np.array([[np.cos(theta), 0.0, np.sin(theta)],
                                  [0.0, 1.0, 0.0],
                                  [-np.sin(theta), 0.0, np.cos(theta)]])
    return rotation_matrix_Y


def rotZ(psi):
    '''
    rotation matrix around the Z axis (yaw)
    '''
    psi = np.deg2rad(psi)
    rotation_matrix_Z = np.array([[np.cos(psi), -np.sin(psi), 0.0],
                                  [np.sin(psi), np.cos(psi), 0.0],
                                  [0.0, 0.0, 1.0]])
    return rotation_matrix_Z


def hmRPYG(roll, pitch, yaw, trans):
    '''
    Generates a homogeneous matrix based on the rotation from the ground frame.
    returns a 4x4 homogeneous matrix of the RPY order.
    trans should be a vector of 3 entries.
    '''

    hmRPY = np.zeros((4, 4)).astype('float')  # init a 4x4 homogeneous matrix
    rotRPY = rotZ(yaw).dot(rotY(pitch)).dot(rotX(roll))
    hmRPY[:3, :3] = rotRPY
    hmRPY[:3, -1] = trans.flatten()
    hmRPY[-1, -1] = 1.0
    return hmRPY


def hmRPYP(roll, pitch, yaw, trans):
    '''
    Generates a homogeneous matrix based on the rotation from the previous intermediate frame.
    returns a 4x4 homogeneous matrix of the RPY order.
    trans should be a vector of 3 entries.
    '''
    hmRPY = np.zeros((4, 4)).astype('float')  # init a 4x4 homogeneous matrix
    rotRPY = rotX(roll).dot(rotY(pitch)).dot(rotZ(yaw))  # the rotation matrix based on the intermediate frame.
    hmRPY[:3, :3] = rotRPY
    hmRPY[:3, -1] = trans.flatten()
    hmRPY[-1, -1] = 1.0
    return hmRPY


def drawGround(pose, ax, label):
    '''
    draw the 3D pose with respect to the ground coordinates
    input: pose is the 4x4 array
    rotation order: RPY, with respect to the ground coords
    '''

    if(pose.shape != (4, 4)):
        print("Wrong pose shape, exited")
        return

    newOrigin = pose[:-1, -1]  # the translation vector of pose, 3x1
    groundUnivVecs = np.vstack((groundCoords * 0.05, np.ones((1, 3))))  # 4x3, padding
    newUnitVecs = np.dot(pose, groundUnivVecs)[:-1, :]  # 4x3 -> 3x3
    newCoordX = np.vstack((newOrigin, newUnitVecs[:, 0])).T  # 3x2
    newCoordY = np.vstack((newOrigin, newUnitVecs[:, 1])).T
    newCoordZ = np.vstack((newOrigin, newUnitVecs[:, 2])).T

    # draw the 3 unit vectors
    plt.ion()
    ax.plot(newCoordX[0, :], newCoordX[1, :], newCoordX[2, :], color = 'r', label="X")
    ax.plot(newCoordY[0, :], newCoordY[1, :], newCoordY[2, :], color = 'g', label="Y")
    ax.plot(newCoordZ[0, :], newCoordZ[1, :], newCoordZ[2, :], color = 'b', label="Z")

    # text
    ax.text(newOrigin[0], newOrigin[1], newOrigin[2] - 0.03, label, color = 'black')
    plt.show()
    plt.pause(0.02)


def drawRigidBody(vertices, ax):
    # vertices is the 8 vertices of the robot rigid body.
    links = [[0, 1], [1, 2], [2, 3], [3, 0],
            [4, 5], [5, 6], [6, 7], [7, 4],
            [0, 4], [1, 5], [2, 6], [3, 7]]
    vertices = vertices[:3, :].T  # should be 8x3 if the rigid body is a rectangular prism.
    for link in links:
        ax.plot3D(*zip(*vertices[link]), color="k", linewidth = 0.3)


class robot():
    def __init__(self, hm, ax, landmarks): # hm is the 4x4 homogeneous matrix, for different rotation orders.
        self.body_length = 0.20  #  meter
        self.body_width = 0.10  #  meter
        self.body_thickness = 0.03  #  meter
        # the centroid of the robot, also the origin of the rigid body frame.
        # Can be adjusted to comply with the actual turning center.
        self.center = np.array([0.0, 0.0, 0.0])
        # To store the vertices of the robot body as a rectangular prism.
        # the first column is the coordinates of the left-front corner of the rigid body,
        # starting from left-front, clockwise; from upper to bottom surface.
        # Under the rigid body frame. Size is 4x8.
        self.body_vertices = np.array([[0.5 * self.body_length - self.center[0], 0.5 * self.body_length - self.center[0], - 0.5 * self.body_length - self.center[0], - 0.5 * self.body_length - self.center[0],
                                        0.5 * self.body_length - self.center[0], 0.5 * self.body_length - self.center[0], - 0.5 * self.body_length - self.center[0], - 0.5 * self.body_length - self.center[0]],  # x coordinate
                                       [0.5 * self.body_width - self.center[1], - 0.5 * self.body_width - self.center[1], - 0.5 * self.body_width - self.center[1], 0.5 * self.body_width - self.center[1],
                                        0.5 * self.body_width - self.center[1], - 0.5 * self.body_width - self.center[1], - 0.5 * self.body_width - self.center[1], 0.5 * self.body_width - self.center[1]],  # y coordinate
                                       [0.5 * self.body_thickness - self.center[2], 0.5 * self.body_thickness - self.center[2], 0.5 * self.body_thickness - self.center[2], 0.5 * self.body_thickness - self.center[2],
                                        - 0.5 * self.body_thickness - self.center[2], - 0.5 * self.body_thickness - self.center[2], - 0.5 * self.body_thickness - self.center[2], - 0.5 * self.body_thickness - self.center[2]],  # z coordinate
                                       [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]])   # padding for computation.
        self.body_verticesGround = self.body_vertices  # in the ground frame.
        self.measurement = np.zeros(6).astype('float')  # init the measurement of each update step.
        self.control = np.zeros(2).astype('float')  # init the 2D control input: translation along the X+ axis, delta-yaw.
        self.odometry = np.zeros(2).astype('float') # init the 2D odometry based on IMU.
        self.hm = hm  # pass the homogeneous matrix calculation func.
        self.initialEstimate = np.zeros(6).astype('float')  # init the initail estimate of pose: roll, pitch, yaw, x, y, z.
        self.updatedEstimate = np.zeros(6).astype('float')
        self.ax = ax  # the plot axis.
        self.cam2body = np.array([-90, 0, 90, 0, 0, 0.15])  # in the camera frame, RPYP. Default: np.array([-90, 0, 90, 0, 0, 0.10])
        self.landmarks = landmarks  # nx7 array.
        self.lastYaw = 0.0
        self.lastStamp = 0.0
        self.trajectoryNo = 0

    def forward(speed=50):
        dataCMD = json.dumps({'var':"move", 'val':1})
        ser.write(dataCMD.encode())
        print('robot-forward')
    
    def stopLR():
        dataCMD = json.dumps({'var':"move", 'val':6})
        ser.write(dataCMD.encode())
        print('robot-stop')

    def stopFB():
        dataCMD = json.dumps({'var':"move", 'val':3})
        ser.write(dataCMD.encode())
        print('robot-stop')	

    def poseUpdate(self):
        # TODO: implement EKF. For now, finish a demo to update pose based on pseudo control.
        self.initialEstimate = self.updatedEstimate

        # (optional) draw the pose of the center of robot.
        drawGround(self.hm(*self.updatedEstimate[:3], self.updatedEstimate[3:]), self.ax, "")

        # (optional) update the rigid body vertices.
        # self.body_verticesGround = self.hm(*self.updatedEstimate[:3], self.updatedEstimate[3:]).dot(self.body_vertices)
        # drawRigidBody(self.body_verticesGround, self.ax)

    def measurementUpdate(self, results, useCalibration = True):
        # TODO: map to the body of dog.

        # handle multiple tags: tags -> poses from each tag -> elimitate the craziest one -> average over the rest.
        # init storage.
        translations = np.array([])
        eulerAngles = np.array([])

        # iterater over each tag's result.
        for idx in np.arange(0, len(results), 4):
            # homo is the homogeneous matrix returned by the apriltag detector.
            # get the correct rotation from the tags.
            homo = results[idx + 1]
            # print(homo)
            tagID = results[idx].tag_id
            # print(tagID)
            # decide which board the robot is looking at.
            self.trajectoryNo = np.where(tagID == tagGroups)[0][0]
            # print("self.trajectoryNo = ", self.trajectoryNo)
            '''
            if tagID not in self.landmarks[:, 0]:
                continue
            '''
            rotCamera = R.from_matrix(homo[:3, :3])
            eulerCamera = rotCamera.as_euler('zyx', degrees = True)
            transCamera = homo[:-1, -1].flatten()
            transCamera[0] *= -1
            # Add the optional calibration function.
            if useCalibration:
                transCamera = calibratePose2D(eulerCamera[1], transCamera)
            rotCamera = rotY(- eulerCamera[2]).dot(rotX(- eulerCamera[1])).dot(rotZ(eulerCamera[0]))  # convert to standard zxy rotmat.
            hmCamera = np.zeros((4, 4)).astype('float')
            hmCamera[:3, :3] = rotCamera
            hmCamera[:-1, -1] = transCamera
            hmCamera[-1, -1] = 1.0
            bodyPoseInTagFrame = np.dot(hmCamera, hmRPYP(*self.cam2body[:3], self.cam2body[3:]))
            targetTagPose = self.landmarks[self.landmarks[:, 0] == tagID, 1:][0]
            poseGround = np.dot(hmRPYP(*targetTagPose[:3], targetTagPose[3:]), bodyPoseInTagFrame)  # Tags use RPYP pose.
            rotGround = R.from_matrix(poseGround[:3, :3])
            translation = poseGround[:3, -1]
            # self.measurement[:3] = rotGround.as_euler('xyz', degrees = True)  # decompose using RPYG rule.
            # self.measurement[3:] = translation
            translations = np.append(translations, translation)
            eulerAngles = np.append(eulerAngles, rotGround.as_euler('xyz', degrees = True))
            

        translations = translations.reshape(-1, 3)
        eulerAngles = eulerAngles.reshape(-1, 3)
        # print(translations)
        # print(eulerAngles)

        numResults = (idx + 4) // 4
        # print("numResults: ", numResults)

        # eliminate trash data (the one deviates most from the mass center) for each dimension.
        if numResults >= 3: 
            translationsAvg = np.average(translations, axis=0)
            eulerAnglesAvg = np.average(eulerAngles, axis=0)
            transMaxIdx = np.argmax(translations - translationsAvg, axis=0)
            eulerAnglesMaxIdx = np.argmax(eulerAngles - eulerAnglesAvg, axis=0)
            translations[transMaxIdx, np.arange(0, translations.shape[1])] = 0  # nullify the deviated data.
            eulerAngles[eulerAnglesMaxIdx, np.arange(0, eulerAngles.shape[1])] = 0
            # average the rest of the data in each dimension.
            translations = np.sum(translations, axis=0) / (numResults - 1)
            eulerAngles = np.sum(eulerAngles, axis=0) / (numResults - 1)
        
        if numResults == 2:
            # average the rest of the data in each dimension.
            translations = np.sum(translations, axis=0) / 2
            eulerAngles = np.sum(eulerAngles, axis=0) / 2

        # print(eulerAngles)
        # print(translations)
        self.measurement[:3] = eulerAngles.flatten()
        self.measurement[3:] = translations.flatten()
        # print(self.measurement)


    def odometryUpdate(self, stamp, dx, yaw):
        # update the initial estimate.
        # the updatedEstimate refreshes in every 2~3 seconds.
        # TODO: wrap it in a insulated thread to constantly update initialEstimate from IMU serial.
        
        if(stamp != None):
            stamp = float(stamp)
            if(self.lastStamp == 0.0):
                dt = 0.0
                self.lastStamp = stamp
            else:
                dt = stamp - self.lastStamp
                self.lastStamp = stamp
        else:
            dt = 0.0
        
        if(yaw == None):
            dyaw = 0.0
        else:
            yaw = float(yaw)
            dyaw = yaw - self.lastYaw
            self.lastYaw = yaw
        
        if(dx == None):
            dx = 0.0
        else:
            dx = float(dx)
        
        trans = np.array([dx, 0, 0])
        print("odoUpdate: ", dyaw, dx)
        initialEstimateHM = np.dot(self.hm(0, 0, dyaw, trans), self.hm(*self.initialEstimate[:3], self.initialEstimate[3:]))
        print(initialEstimateHM)
        self.initialEstimate[3:] = initialEstimateHM[:-1, -1].flatten()
        self.initialEstimate[:3] = R.from_matrix(initialEstimateHM[:3, :3]).as_euler('xyz', degrees = True)
        print("Initial Estimate: ", self.initialEstimate)

    def controlUpdate(self, ctrl):
        # demo for displaying the 3D drawing.
        self.control = ctrl
        # update the init guess of pose (ground truth).
        # consider move the Gaussian noise here (forward noise).
        translation = np.array([[self.control[0], 0.0, 0.0, 1]]).T  # in the rigid body frame
        self.initialEstimate[3:] = self.hm(*self.updatedEstimate[:3], self.updatedEstimate[3:]).dot(translation).flatten()[:-1]  # x, y, z in the world frame.
        self.initialEstimate[2] = self.updatedEstimate[2] + self.control[1]  # only change yaw.

    def brickPosePropose(self):
        # TODO: generate a proposal for brick position, to be passed to the brick class.
        brickPoseBodyFrame = np.array([0, 0, 0, 0, 0, 0])  # ignore the actuation mechanism.
        poseProposal = (self.hm(*self.updatedEstimate[:3], self.updatedEstimate[3:])).dot(brickPoseBodyFrame)
        


class landmarks():
    def __init__(self, hm, poses, ax):
        '''
        Init the ground truth locations of the Apriltags.
        TODO: How to specify the initial location when the robot class is instanciated? Relative position.
        '''
        self.hm = hm  # specify the type of homegeneous matrix.
        self.poses = poses  # poses corresponding to the homogeneous matrix. Shape: nx7, 7 = tagID + 6 poses, n = No. of tags.
        self.ax = ax  # drawing axis.
        self.tagSize = 0.10
        self.tagThickness = 0.02
        self.vertices = np.array([[0.5 * self.tagSize, 0.5 * self.tagSize, - 0.5 * self.tagSize, - 0.5 * self.tagSize, 
                                        0.5 * self.tagSize, 0.5 * self.tagSize, - 0.5 * self.tagSize, - 0.5 * self.tagSize],
                                       [0.5 * self.tagSize, - 0.5 * self.tagSize, - 0.5 * self.tagSize, 0.5 * self.tagSize, 
                                        0.5 * self.tagSize, - 0.5 * self.tagSize, - 0.5 * self.tagSize, 0.5 * self.tagSize],
                                       [0.5 * self.tagThickness, 0.5 * self.tagThickness, 0.5 * self.tagThickness, 0.5 * self.tagThickness, 
                                        - 0.5 * self.tagThickness, - 0.5 * self.tagThickness, - 0.5 * self.tagThickness, - 0.5 * self.tagThickness],
                                       [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]])  # padding for computation.


class brickMap():
    def __init__(self, hm, ax) -> None:
        '''
        Init the class to store brick model and update the brick map.
        '''
        self.hm = hm
        self.ax = ax
        self.brickLength = 0.40 # meter
        self.brickWidth = 0.20
        self.brickThickness = 0.015
        self.brickVertices = np.array([[0.5 * self.brickLength, 0.5 * self.brickLength, - 0.5 * self.brickLength, - 0.5 * self.brickLength, 
                                        0.5 * self.brickLength, 0.5 * self.brickLength, - 0.5 * self.brickLength, - 0.5 * self.brickLength],
                                       [0.5 * self.brickWidth, - 0.5 * self.brickWidth, 0.5 * self.brickWidth, - 0.5 * self.brickWidth, 
                                        0.5 * self.brickWidth, - 0.5 * self.brickWidth, 0.5 * self.brickWidth, - 0.5 * self.brickWidth],
                                       [0.5 * self.brickThickness, 0.5 * self.brickThickness, 0.5 * self.brickThickness, 0.5 * self.brickThickness, 
                                        - 0.5 * self.brickThickness, - 0.5 * self.brickThickness, - 0.5 * self.brickThickness, - 0.5 * self.brickThickness],
                                       [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]])  # padding for computation.
        # self.map = np.zeros(6).astype('float')  # to store the poses of the bricks as maps.
        self.map = []  # to utilize the append attribute of lists. len = No. of bricks.

    def place(self, pose):  # pose is a list len = 6.
        # TODO: detect collision and layer.
        if self._viabilityDetect(pose) != True:  # if the pose of brick is viable.
            self.map.append(pose)
            drawGround(hmRPYG(*pose[:3], pose[3:]), self.ax, "Brick ".join(str(len(self.map + 1))))  # len(list) returns the rows of a list (first dim).
            drawRigidBody(hmRPYG(*pose[:3], pose[3:]).dot(self.brickVertices), self.ax)
            return True  #  to be passed to the robot class.
        else:
            print("Invalid Brick Pose")
            return False  #  to be passed to the robot class.

    def _viabilityDetect(self, pose) -> bool:  # pose is a list len = 6.
        # TODO: the rule check function before placing bricks.
        return True

# WiFi data TX function.
class UDPSender():
    def __init__(self, pc_ip, pc_port) -> None:
        self.pc_ip = pc_ip
        self.pc_port = pc_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, message, verbose = False):  # message is a np.array of the robot pose.
        # serialize the array.
        serialized_array = pickle.dumps(message)
        self.sock.sendto(serialized_array, (self.pc_ip, self.pc_port))
        if verbose:
            print("Data Sent to IP {}, Port {}".format(self.pc_ip, self.pc_port))

    def close(self):
        self.sock.close()

# ---- Create a figure object ----
fig0 = plt.figure(0)
ax_fig0 = fig0.add_subplot(111, projection='3d')
ax_fig0.set_xlabel('X')
ax_fig0.set_ylabel('Y')
ax_fig0.set_zlabel('Z')

ax_fig0.set_xlim([-0.5, 1.0])
ax_fig0.set_ylim([-0.75, 0.75])
ax_fig0.set_zlim([0, 1.5])

# ---- Initialize the tags as an object ----
boardBiasesX = np.array([0.005, 0.025, 0.007, 0.0])  # 0.018
boardBiasesY = np.array([0.02, -0.025, 0.0, 0.02])
boardBiasesYaw = np.array([-1.5, -1, 0, 0])

poseTags = np.array([[4, 90, -90, 0, 0.994 + 0.265 - 0.10, 0, 0.055 + 0.172/2],
                 [3, 90, -90, 0, 0.994 + 0.265 - 0.10, 0.172 + 0.022, 0.055 + 0.172/2],
                 [5, 90, -90, 0, 0.994 + 0.265 - 0.10, -(0.172 + 0.022), 0.055 + 0.172/2],
                 [7, 90, 0, 0, 0.994 - 0.20, 0.995 + 0.267 - 0.10, 0.055 + 0.172/2],
                 [6, 90, 0, 0, 0.994 - 0.20 + 0.172 + 0.022, 0.995 + 0.267 - 0.10, 0.055 + 0.172/2],
                 [1, 90, 0, 0, 0.994 - 0.20 - 0.172 - 0.022, 0.995 + 0.267 - 0.10, 0.055 + 0.172/2],
                 [0, 90, 90, 0, -0.376, 1.002 - 0.20 + 0.172 + 0.022, 0.055 + 0.172/2],
                 [8, 90, 90, 0, -0.376, 1.002 - 0.20 - (0.172 + 0.022), 0.055 + 0.172/2],
                 [9, 90, 90, 0, -0.376, 1.002 - 0.20, 0.055 + 0.172/2],
                 [2, 90, 180, 0, 0.172 + 0.022, -0.35, 0.055 + 0.172/2],
                 [10, 90, 180, 0, 0.0, -0.35, 0.055 + 0.172/2],
                 [11, 90, 180, 0, -(0.172 + 0.022), -0.35, 0.055 + 0.172/2]])

# adjust the biases of tags.
poseTags[0:3, 4] += boardBiasesX[0]
poseTags[3:6, 5] += boardBiasesX[1]
poseTags[6:9, 4] += boardBiasesX[2]
poseTags[9:12, 5] += boardBiasesX[3]

poseTags[0:3, 5] += boardBiasesY[0]
poseTags[3:6, 4] += boardBiasesY[1]
poseTags[6:9, 5] += boardBiasesY[2]
poseTags[9:12, 4] += boardBiasesY[3]

'''
poseTags[0:3, 2] += boardBiasesYaw[0]
poseTags[3:6, 2] += boardBiasesYaw[1]
poseTags[6:9, 2] += boardBiasesYaw[2]
poseTags[9:12, 2] += boardBiasesYaw[3]
'''

myTags = landmarks(hmRPYP, poseTags, ax_fig0)  # tags use RPYP pose.
'''
for _, pose in enumerate(myTags.poses):
        drawGround(myTags.hm(*pose[1:4], pose[4:]), myTags.ax, "Tag "+str(int(pose[0])))
        drawRigidBody(myTags.hm(*pose[1:4], pose[4:]).dot(myTags.vertices), myTags.ax)
'''
# ---- Instantiate the robot class ----
myRobot = robot(hmRPYG, ax_fig0, poseTags)
poseRecord = np.zeros((1, 6))

# data sender init.
PC_IP = "10.50.9.219"  # should be updated DAILY!!!
PC_Port = 52000
poseSender = UDPSender(PC_IP, PC_Port)

# store the previous pose (control sending speed)
previousPose = np.zeros(6).astype('float')

# a simpler trajectory, in the world frame.
trajectory = np.array([[0.0, 0.994 - 0.20, 0.994 - 0.20, 0.0, 0.0],  # x coordinate
                       [0.0, 0.0, 0.995 - 0.20, 0.995 - 0.20, 0.0],  # y coordinate
                       [0.0, 0.0, 0.0, 0.0, 0.0]])  # z coordinate

# tag grouping, each row represents the tagIDs on the same board.
tagGroups = np.array([[3, 4, 5],
                      [1, 7, 6],
                      [8, 9, 0],
                      [2, 10, 11]])

def checkTurning(brickNo, pose, trajectory):
    corner = trajectory[:, brickNo + 1]
    yaw = pose[1]
    targetVector = corner - pose[3:]
    targetVector = hmRPYP(0, 0, -90*brickNo, np.array([0, 0, 0]))[:3, :3].dot(targetVector.reshape(-1, 1))[:-1]
    return yaw, targetVector

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

    try:  
        # myRobot.forward()


        while(1):
            # myRobot.odometryUpdate(*readIMU())
            if avp.resultsGlobal != []:
                myRobot.measurementUpdate(avp.resultsGlobal)
                if not np.all(myRobot.measurement.copy() == previousPose.copy()):  
                    # drawGround(hmRPYG(*myRobot.measurement[:3], myRobot.measurement[3:]), ax_fig0, "")
                    # print("Measurement at {} : {}".format(time.time(), myRobot.measurement))
                    poseSender.send(np.append(time.time(), myRobot.measurement.copy()))  # send the pose array.
                    previousPose = myRobot.measurement.copy()  # store the previous pose.
                    yawInTag, targetVector = checkTurning(myRobot.trajectoryNo, myRobot.measurement, trajectory)
                    targetVector *= 100  # transform to cm
                    yawInTag += boardBiasesYaw[myRobot.trajectoryNo]
                    turningPose = np.append(yawInTag, targetVector.flatten())
                    print(turningPose)
                    fl.robotPose = turningPose
                    # print(yawInTag, targetVector)
                    # np.savetxt('robotPose.csv', turningPose, delimiter=',')
                    # print(turningPose)
                    time.sleep(0.1)
                    if not walkThreadRunning:
                        # from Yao's code.
                        fl.walkThread.start()
                        print('walk Thread start')
                        time.sleep(2)
                        walkThreadRunning = True
            time.sleep(0.1)


    except KeyboardInterrupt:
        
        # myRobot.stopFB()
        # Kill the threads.
        avp.aptIsRunning = False
        walkThreadRunning = False
        apriltagDetectionThread.join()
        fl.walkThread.join()
        # odometryReadIsRunning = False
        # odometryReadThread.join()
        print('STOOOOOOOOOOOOOOOOOOOOOP')
        fl.iswalk=0
        poseSender.close()
        print("exited main")
        plt.close()
        ser.close()
