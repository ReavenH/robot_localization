# define the ground coordinates:
# X: pointing forwards; Y: pointing left; Z: pointing upwards
# each column: x, y, z point coords,
import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
import serial
from scipy.spatial.transform import Rotation as R
from scipy.spatial.distance import cdist
import time
import os
import json
import socket
import re
import pickle
from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
import platform
import cv2
from scipy.io import savemat, loadmat
from scipy.spatial import distance
from collections import deque

if platform.system() == "Linux":  # if on raspberry pi
    # instantiate gpio control for the servos.
    import subprocess
    subprocess.run('sudo pigpiod', shell=True, check=True)
    import pigpio
    pi = pigpio.pi()

    
    # from WAVESHARE, establish serial connection.
    def connect_serial(port, baudrate):
        while True:
            try:
                ser = serial.Serial(port, baudrate)
                print("Serial is connected: {}".format(ser.is_open))
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
    

    def readIMU(ser):

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

    '''
    plt.show()
    plt.pause(0.02)  # default 0.02
    '''

def drawLineOG(start, end):
    '''
    Draw a line segment using OpenGL.
    Inputs: 
        start: the start point x, y, z;
        end: the end point x, y, z.
    Output: None.
    '''
    glBegin(GL_LINES)
    glColor3f(0.1, 0.1, 0.1)  # color of the line.
    glVertex3f(*start)
    glVertex3f(*end)
    glEnd()

def drawGroundOG(pose, scale = 1.0):
    '''
    draw the 3D pose with respect to the ground coordinates
    input: pose is the 4x4 array
    rotation order: RPY, with respect to the ground coords
    using OpenGL
    '''

    if(pose.shape != (4, 4)):
        print("Wrong pose shape, exited")
        return

    newOrigin = pose[:-1, -1]  # the translation vector of pose, 3x1
    groundUnivVecs = np.vstack((groundCoords * 0.05 * scale, np.ones((1, 3))))  # 4x3, padding
    newUnitVecs = np.dot(pose, groundUnivVecs)[:-1, :]  # 4x3 -> 3x3
    newCoordX = np.vstack((newOrigin, newUnitVecs[:, 0]))  # 2x3
    newCoordY = np.vstack((newOrigin, newUnitVecs[:, 1]))
    newCoordZ = np.vstack((newOrigin, newUnitVecs[:, 2]))

    glLineWidth(3.0)
    # draw lines with OpenGL
    glBegin(GL_LINES)
    glColor3f(1.0, 0.0, 0.0) # R
    glVertex3dv(newCoordX[0])
    glVertex3dv(newCoordX[1])
    glColor3f(0.0, 1.0, 0.0) # G
    glVertex3dv(newCoordY[0])
    glVertex3dv(newCoordY[1])
    glColor3f(0.0, 0.0, 1.0) # B
    glVertex3dv(newCoordZ[0])
    glVertex3dv(newCoordZ[1])
    glEnd()

def drawRigidBody(vertices, ax):
    # vertices is the 8 vertices of the robot rigid body.
    links = [[0, 1], [1, 2], [2, 3], [3, 0],
            [4, 5], [5, 6], [6, 7], [7, 4],
            [0, 4], [1, 5], [2, 6], [3, 7]]
    vertices = vertices[:3, :].T  # should be 8x3 if the rigid body is a rectangular prism.
    for link in links:
        ax.plot3D(*zip(*vertices[link]), color="k", linewidth = 0.8)

def drawRigidBodyOG(vertices):
    # vertices is the 8 vertices of the robot rigid body.
    links = [[0, 1], [1, 2], [2, 3], [3, 0],
            [4, 5], [5, 6], [6, 7], [7, 4],
            [0, 4], [1, 5], [2, 6], [3, 7]]
    vertices = vertices[:3, :].T  # should be 8x3 if the rigid body is a rectangular prism.

    # draw with OpenGL
    glLineWidth(1.5)
    glBegin(GL_LINES)
    for link in links:
        for vertex in link:
            glColor3f(0.0, 0.0, 0.0)
            glVertex3dv(vertices[vertex])
    glEnd()

# the experimental class for the flipping linkage for placing a brick.
class flipLinkage():
    def __init__(self, linkageS, linkageA, linkageB, linkageW) -> None:
        '''
        The linkage B (longer one) is connected to a servo, while the linkage A is not.
        Refer to the Chebyshev Linkage Model. The inverse kinematics may not be solvable.
        linkageW > linkageS, linkageA > linkageS and linkageA must not collide with the robot front.
        '''
        self.linkageA = linkageA  # meters.
        self.linkageW = linkageW  # meters.
        self.linkageS = linkageS  # meters.
        self.linkageB = linkageB # meters.
        self.E_PI = 180 / np.pi
        # TODO use functions to auto calculate.
        self.betaLim1 = 55 # degs.
        self.betaLim2 = 20 
        self._inverseKinematics(0)  # init the joint angles.
    
    def kinematics(self, angleBeta):
        '''
        Computes the joint angles of the linkages, given the driving servo angle.
        Based on the vector constant rule.
        '''
        self.angleBeta = angleBeta

        # when joint A is above linkageB.
        if self.angleBeta >= self.betaLim1:
            coeffs = np.array([[self.linkageA, self.linkageW, self.linkageS + self.linkageB * np.cos(self.angleBeta / 180 * np.pi)],
                               [self.linkageA, self.linkageW, self.linkageB * np.sin(self.angleBeta / 180 * np.pi)]])

            # init symbol variables.
            alpha, theta = sp.symbols('alpha theta')

            # list the two equations.
            eqCos = sp.Eq(coeffs[0][0] * sp.cos(alpha) + coeffs[0][1] * sp.cos(theta), coeffs[0][2])
            eqSin = sp.Eq(coeffs[1][0] * sp.sin(alpha) + coeffs[1][1] * sp.sin(theta), coeffs[1][2])

            # solve alpha first.
            cosTheta = (coeffs[0][-1] - coeffs[0][0] * sp.cos(alpha)) / coeffs[0][1]
            sinTheta = (coeffs[1][-1] - coeffs[1][0] * sp.sin(alpha)) / coeffs[1][1]

            # solve self.angleAlpha using triangulation.
            eqTriangulation = sp.Eq(cosTheta ** 2 + sinTheta ** 2, 1)
            self.angleAlpha = np.array(sp.solve(eqTriangulation, alpha))
            self.angleAlpha = self.angleAlpha[(self.angleAlpha <= np.pi / 2) & (self.angleAlpha >= - np.pi / 2)][0]

            # solve self.angleTheta by plugging in alpha.
            cosThetaSol = cosTheta.subs(alpha, self.angleAlpha)
            sinThetaSol = sinTheta.subs(alpha, self.angleAlpha)
            self.angleTheta = sp.atan2(sinThetaSol, cosThetaSol).evalf() * self.E_PI
            self.angleAlpha = self.angleAlpha * self.E_PI

        elif self.angleBeta < self.betaLim1 and self.angleBeta >= self.betaLim2:
            pass


        print("Updated angleAlpha: {}".format(self.angleAlpha))
        print("Updated angleTheta: {}".format(self.angleTheta))
        print("Updated angleBeta: {}".format(self.angleBeta))
        

    def _calculateBetaLimits(self):
        pass

    def _inverseKinematics(self, targetAngle):
        '''
        The targetAngle is the tilt angle of the board in the body frame.
        '''
        if targetAngle == 180.0:
            '''
            When the board above is parallel to the body.
            Solve using similar triangles.
            '''
            self.angleTheta = 180.0
            OB = self.linkageB * (self.linkageW / (self.linkageW + self.linkageS))
            OA = self.linkageA * (self.linkageW / (self.linkageW + self.linkageS))
            self.angleAlpha = np.arccos((OA ** 2 + self.linkageW ** 2 - OB ** 2) / (2 * OA * self.linkageW)) * self.E_PI
            self.angleBeta = 180 - np.arccos((OB ** 2 + self.linkageW ** 2 - OA ** 2) / (2 * OB * self.linkageW)) * self.E_PI
        elif targetAngle > 0.0 and targetAngle < 90.0:
            print("IK not available at theta={}!".format(targetAngle))
            pass
        elif targetAngle == 0.0:
            self.angleTheta = 0.0
            # similar triangles.
            m = (self.linkageS * self.linkageB) / (self.linkageW - self.linkageS)
            n = (self.linkageS * self.linkageA) / (self.linkageW - self.linkageS)
            OB = self.linkageB + m
            OA = self.linkageA + n
            print(OB, OA)
            print((self.linkageW ** 2 + OB ** 2 - OA ** 2) / (2 * self.linkageW * OB))
            self.angleBeta = np.arccos((self.linkageW ** 2 + OB ** 2 - OA ** 2) / (2 * self.linkageW * OB)) * self.E_PI
            print(self.angleBeta)
            self.angleAlpha = (np.pi - np.arccos((n ** 2 + self.linkageS ** 2 - m ** 2) / (2 * self.linkageS * n))) * self.E_PI

        print("IK angleAlpha: {}".format(self.angleAlpha))
        print("IK angleTheta: {}".format(self.angleTheta))
        print("IK angleBeta: {}".format(self.angleBeta))

    def drawLinkages(self):
        pass

class robot():
    def __init__(self, hm, ax, landmarks, ser, config=None, vidsrc = 1): # hm is the 4x4 homogeneous matrix, for different rotation orders.
        self.body_length = 0.18725  #  meter, actual size including the camera.
        self.body_width = 0.06533  #  meter, actual size.
        self.body_thickness = 0.0376  #  meter, actual size without the RPi and lid.
        # the centroid of the robot, also the origin of the rigid body frame.
        # Can be adjusted to comply with the actual turning center. If being adjusted, it should indicate the translation from the previous centroid to the new.
        self.center = np.array([0.0, 0.0, 0.0])
        self.bodyPose = np.zeros(6).astype('float')  # pose of body center in the ground frame. Uses RPYG order.
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
        self.body_verticesGround = hmRPYG(*self.bodyPose[:3], self.bodyPose[3:]).dot(self.body_vertices)  # in the ground frame. Initialization.
        self.measurement = np.zeros(6).astype('float')  # init the measurement of each update step. Uses RPYG order.
        self.control = np.zeros(2).astype('float')  # init the 2D control input: translation along the X+ axis, delta-yaw.
        self.odometry = np.zeros(2).astype('float') # init the 2D odometry based on IMU.
        self.hm = hm  # pass the homogeneous matrix calculation func.
        self.initialEstimate = np.zeros(6).astype('float')  # init the initail estimate of pose: roll, pitch, yaw, x, y, z.
        self.updatedEstimate = np.zeros(6).astype('float')
        if ax is not None: 
            self.ax = ax  # the plot axis for matplotlib.
        self.cam2body = np.array([-90, 0, 90, 0, 0, 0.15])  # in the camera frame, RPYP. Default: np.array([-90, 0, 90, 0, 0, 0.10])
        self.landmarks = landmarks  # nx7 array.
        self.lastYaw = 0.0
        self.lastStamp = 0.0
        self.trajectoryNo = 0
        # Linkage Model Params for Solving IK.
        self.linkageA = 0.04 # meters. The linkage connected with the servos. Two identical ones for each leg.
        self.linkageB = 0.04 # meters. The supporting linkage attached to the linkage A on the front servo of each leg.
        self.linkageC = 0.0398153 # meters. The upper segment of the curved leg attached to the linkage A on the rear servo of each leg.
        self.linkageD = 0.0317750 # meters. The lower segment of the curved leg.
        self.linkageE = 0.0308076 # meters. The segment perpendicular to linkageD, attaching to the foot.
        self.linkageS = 0.0122 # meters. The distance between to axes of servos.
        self.linkageW = 0.01915 # meters. The offset of the leg plane from the origin of the linkage model frame.
        # Pre-render some frequently used variables. (Refer to the WAVEGO Arduino Sourcecode)
        self.LAxLA = self.linkageA ** 2
        self.LBxLB = self.linkageB ** 2
        self.LWxLW = self.linkageW ** 2
        self.LExLE = self.linkageE ** 2
        self.LAxLA_LBxLB = self.LAxLA - self.LBxLB
        self.LBxLB_LAxLA = self.LBxLB - self.LAxLA
        self.L_CD = (self.linkageC + self.linkageD) ** 2
        self.LAx2 = 2 * self.linkageA
        self.LBx2 = 2 * self.linkageB
        self.E_PI = 180 / np.pi  # for rad to deg conversion.
        self.LSs2 = self.linkageS / 2
        self.aLCDE = np.arctan2((self.linkageC + self.linkageD), self.linkageE)
        self.sLEDC = np.sqrt(self.linkageE ** 2 + (self.linkageD + self.linkageC) ** 2)
        self.angleEpsilon = (np.pi / 2 - self.aLCDE) * self.E_PI  # in degs, the fixed angle opposing to linkageE.
        # The offsets for the 4 origins of the leg linkage models' frames, from the body centroid. For 3D coodinate propagation.
        self.linkageFrameOffsets = np.array([[(0.041 + 0.0096) - self.center[0], 0.5 * self.body_width - self.center[1] - 0.006, 0.5 * self.body_thickness - self.center[2] - 0.01145],  # offsets for the 4 leg origins, leg 1 (left-front).
                                       [- (0.041 + 0.0096) - self.center[0], 0.5 * self.body_width - self.center[1] - 0.006, 0.5 * self.body_thickness - self.center[2] - 0.01145], # leg 2 (left-rear).
                                       [(0.041 + 0.0096) - self.center[0], - 0.5 * self.body_width - self.center[1] + 0.006, 0.5 * self.body_thickness - self.center[2] - 0.01145], # leg 3 (right-front).
                                       [- (0.041 + 0.0096) - self.center[0], - 0.5 * self.body_width - self.center[1] + 0.006, 0.5 * self.body_thickness - self.center[2] - 0.01145]]) # leg 4 (right-rear).
        # The angles in degrees for each linkage.
        # TODO: give them proper initial values.
        self.linkageAngles = np.array([[0, 0, 0, 0 ,0],  # LF leg: [angle-YZPlaneRotation, angle servoFront-A, angle A-B, angle servoRear-A, angle A-C]
                                       [0, 0, 0, 0, 0],  # LR leg
                                       [0, 0, 0, 0, 0],  # RF leg
                                       [0, 0, 0, 0, 0]]).astype('float')  # RR leg
        # Storage of wiggle length (the Euclidian distance from the linkage frame origin to the foot)
        self.wiggleLength = np.array([0, 0, 0, 0]).astype('float')  # one entry for each leg, from leg 1 to leg 4.
        # Storage of servo driving angles.
        self.anglesOutput = np.array([[0, 0, 0],  # leg1: wiggle servo, front servo, rear servo.
                                      [0, 0, 0],
                                      [0, 0, 0],
                                      [0, 0, 0]]).astype('float')
        # The pose of each linkage joint in the ground frame. 
        # Format: [row, pitch, yaw, dx, dy, dz]
        # TODO: give them proper initial values.
        # TODO: maybe add a function to update the following values with self.linkageAngles and those linkage lengths.
        self.linkageCoordinatesGround = np.array([[[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]],  # LF leg: [Joint servoFront-linkageA], [Joint linkageA-B], [Joint servoRear-linkageA], [Joint linkageA-C]
                                                [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]],  # LR leg
                                                [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]],  # RF leg
                                                [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]]).astype('float')  # RR leg
        # initial feet positions
        self.initFeetPos = np.array([[0.0, 0.090, 0.045],
                                     [0.0, 0.090, 0.045],
                                     [0.0, 0.090, 0.045],
                                     [0.0, 0.090, 0.045]])
        self.feetPosControl(self.initFeetPos)

        # check the running platform.
        # 0=Windows, 1=Raspberry Pi, 2=Linux, None=others.
        self.onPlatform = self._checkPlatform()
        print("Robot Class running on Platform {}".format(self.onPlatform))

        # init servo config files.
        self.servoNo = [17, 27, 22]  # BCM of servo PWM pins.
        self.servoCriticalAngles = {}
        self.config = {}
        self.servoDefaultAngles = []
        self.servoAngles = []
        if config != None and self.onPlatform == 1:
            with open(config, 'r', encoding='utf-8') as json_file:
                self.servoCriticalAngles = json.load(json_file)
                self.config = self.servoCriticalAngles
            self.servoDefaultAngles = [self.servoCriticalAngles["linkageUp"], self.servoCriticalAngles["brickUp"], self.servoCriticalAngles["gripperLoose"]]
            self.servoAngles = [self.servoCriticalAngles["linkageUp"], self.servoCriticalAngles["brickUp"], self.servoCriticalAngles["gripperLoose"]]
            # init pins.
            self._servoIOInit(50)
            print("Servos initialized!")

        # composite brickMap class.
        self.brickMap = brickMap(hmRPYG, None)

        # composite the serial class.
        if ser != None:
            self.ser = ser

        # set the resolution/source of the bottom camera.
        self.bottomCamRes = (320, 240) # default (160, 120)
        self.bottomCamSrc = vidsrc  # /video1 

        # store the previous bottom camera poses.
        self.bottomLineCentroid = [0.0, 0.0]
        self.bottomLineYaw = np.array([0.0])
        self.bottomLineYawStraight = 0.0  # it only stores the yaw along the forward direction.
        self.walkDir = 0.0  # in degs.
        self.denoConst = self.bottomCamRes[0] / (1 * np.sin(2 / 180 * np.pi))
        self.previousCircles = 0
        self.nextCircles = 0
        self.lostVision = 0  # -1 if the traces disappeared on the left, 1 for right.
        self.horizontalLimit = 90 # in pixels, will adjust the dog's position for better visibility if the com exceeds this value.
        self.entryDir = np.array([])

        # store the global step from the ESP32.
        self.globalStep = 0.0

        # store the dynamic state..
        self.isMoving = False

        # whether the robot is at a crossing.
        self.atCrossing = False  # True means detects a crossing.
        self.prevCrossing = False  # whether the robot was at a crossing at the previous time step.
        self.atCrossingFIFO = deque([False]*35, maxlen=35)  # a FIFO window to decide whether the robot is at a crossing based on the proportion. Default length is 17.
        self.countCrossing = 0  # to count the number of the crossings that have passed.

        # the motionController's params.
        self.yawTolerance = 4  # the target range of the freeturn movement.
        self.yawThreshold = 8  # the robot will stop and turn when the yaw exceeds this limit.
        self.isturn = False  # the flag whether the robot should continue to freeturn.
        
        # the movement schedular's params.
        # the first will always be F.
        self.path = "FFFFFS"  # F: forward; B: backward; L: left turn 90degs; R: right turn 90degs; S: stop.
        # self.path = "FS"
        self.currentAction = "F"

        # the right leg bias param.
        self.rlb = 0.0
        self.rlbPIDParams = np.array([0.0, 0.0, 0.0])  # the P, I, D parameter for the RLB PID Controller.

        print("Robot Class initialized!")
        
    def _checkPlatform(self):
        system = platform.system()
        if system == "Windows":
            return 0
        elif system == "Linux":
            try:
                with open('/proc/device-tree/model') as model_file:
                    model = model_file.read().lower()
                    if 'raspberry pi' in model:
                        return 1
            except FileNotFoundError:
                pass
            return 2
        else:
            return None

    def _servoIOInit(self, frequency):
        for pin in self.servoNo:
            pi.set_mode(pin, pigpio.OUTPUT)
            pi.set_PWM_frequency(pin, frequency)

    def singleServoCtrl(self, number_servo, angle, speed):
        while angle != self.servoAngles[number_servo]:
            self.servoAngles[number_servo] += min(max(angle-self.servoAngles[number_servo],-speed),speed)
            pi.set_servo_pulsewidth(self.servoNo[number_servo], self.servoAngles[number_servo])
            # time.sleep(0.01)

    def resetPose(self):
        self.singleServoCtrl(0, self.servoDefaultAngles[0], 1/2)
        time.sleep(0.5)
        self.singleServoCtrl(1, self.servoDefaultAngles[1], 1/2)
        time.sleep(0.5)
        self.singleServoCtrl(2, self.servoDefaultAngles[2], 1/2)
        time.sleep(0.5)
        print("Reset Linkage Pose.")

    def openGripper(self):
        print("Open Gripper.")
        self.singleServoCtrl(2, self.servoCriticalAngles["gripperLoose"], 1/2)

    def closeGripper(self):
        print("Close Gripper.")
        self.singleServoCtrl(2, self.servoCriticalAngles["gripperFasten"], 1/2)

    def placeBrickPhase1(self):
        # progress 1~4.
        for i in [1, 2, 3, 4]:
            self.placeBrick(i, verbose=True)

    def placeBrickPhase2(self):
        # progress 5: release brick.
        self.placeBrick(5, verbose=True)

    def placeBrickPhase3(self):
        # progress 6~7.
        for i in [6, 7]:
            self.placeBrick(i, verbose=True)

    def placeBrickPhase4(self):
        # progress 8~9.
        for i in [8, 9]:
            self.placeBrick(i, verbose=True)

    def pushBrick(self, offset, verbose=False):
        # lean forward.
        if verbose:
            print("Leaning Forward.")
        dataCMD = json.dumps({'var':"swing", 'val':offset})  # offset in mm. positive offset -> leaning forward.
        self.ser.write(dataCMD.encode())
        time.sleep(8)  # wait until finish.
        # change the walk clearance to default.
        # self.changeclearance()

    def leanBack(self, offset, verbose=False):
        # reset pose.
        if verbose:
            print("Leaning Backwards.")
        dataCMD = json.dumps({'var':"swing", 'val':-offset})
        self.ser.write(dataCMD.encode())
        time.sleep(5)  # wait until finish.
        # change the walk clearance to default.
        # self.changeclearance()

    def placeBrick(self, progress, verbose=False):
        # primarily called internally by other functions.
        if progress == 1:
            # linkage down.
            if verbose:
                print("Linkage Tilt.")
            self.singleServoCtrl(0, self.servoCriticalAngles["linkageTilt"], 1/2)  # default speed is 1/10.
            time.sleep(0.5)

        if progress == 2:
            # align brick.
            if verbose:
                print("Align Brick.")
            self.singleServoCtrl(1, self.servoCriticalAngles["brickVertical"], 1/20)  # rotate brick board to vertical.
            time.sleep(0.5)
            self.singleServoCtrl(2, self.servoCriticalAngles["gripperAlign"], 1/2)  # gripper open
            time.sleep(1)
            self.singleServoCtrl(2, self.servoCriticalAngles["gripperFasten"], 1/2)  # gripper fasten
            time.sleep(0.5)

        if progress == 3:
            # rotate brick.
            if verbose:
                print("Rotate Brick.")
            self.singleServoCtrl(1, self.servoCriticalAngles["brickDown"], 1/50)  # changed
            time.sleep(1)

        if progress == 4:
            # linkage down.
            if verbose:
                print("Linkage Down.")
            self.singleServoCtrl(0, self.servoCriticalAngles["linkageDown"], 1/10)  # changed
            time.sleep(1)

        if progress == 5:
            # release brick.
            if verbose:
                print("Release Brick.")
            self.singleServoCtrl(2, self.servoCriticalAngles["gripperLoose"], 1/10)
            time.sleep(2)

        if progress == 6:
            # linkage slightly up.
            if verbose:
                print("Linkage Slightly Up.")
            self.singleServoCtrl(0, self.servoCriticalAngles["linkageSlightlyUp"], 1/2)
            time.sleep(1)

        if progress == 7:
            # rotate brickboard.
            if verbose:
                print("Rotate Brickboard.")
            self.singleServoCtrl(1, self.servoCriticalAngles["brickUp"], 1/2)
            time.sleep(0.5)

        if progress == 8:
            # fasten gripper.
            if verbose:
                print("Fasten Gripper.")
            self.singleServoCtrl(2, self.servoCriticalAngles["gripperClose"], 1/2)
            time.sleep(0.5)

        if progress == 9:
            # linkages up.
            if verbose:
                print("Linkage Up.")
            self.singleServoCtrl(0, self.servoCriticalAngles["linkageUp"], 10)

    def triangularwalk(self, degree, distance=40, wait=1.5, token = "Action: Triangular Gait", continuous = True):
        if self.isMoving == False and continuous:
            print("S->M")
            self.startwalknew()
        print('T DEG: ',degree,' DIS: ',distance)
        dataCMD = json.dumps({'var':"TriangularWalk", 'val':degree, 'dis':distance})
        self.ser.write(dataCMD.encode())
        # not necessary to acknowledge when the dog is moving (in the while loop of startwalknew in the ESP32).
        if self.isMoving == False:  
            timeSend = time.time()
            while True:
                if self.ser.in_waiting > 0:
                    ack = self.ser.readline().decode().strip()
                    if ack == token:
                        print("{} received.".format(ack))
                        break
                if time.time() - timeSend > wait:
                    print("Timeout, resending...")
                    self.ser.write(dataCMD.encode())
                    timeSend = time.time()
        if continuous: 
            self.isMoving = True
        # time.sleep(0.1)

    def freeturn(self, degree, wait = 1.5, token = "ActionK: TURNING Once"):
        if self.isMoving == True:
            self.stopwalknew()
        print('FT: ', degree)
        dataCMD = json.dumps({'var':"freeturn", 'val':degree})
        self.ser.write(dataCMD.encode())
        timeSend = time.time()
        while True:
            if self.ser.in_waiting > 0:
                ack = self.ser.readline().decode().strip()
                if ack == token:
                    print("{} received.".format(ack))
                    break
            if time.time() - timeSend > wait:
                print("Timeout, resending...")
                self.ser.write(dataCMD.encode())
                timeSend = time.time()
        self.isMoving = False
        self.stopwalknew()
        # time.sleep(0.1)

    def stopwalknew(self, wait = 1.5, token = "FBStop"):
        dataCMD = json.dumps({'var':"move", 'val':3})
        self.ser.write(dataCMD.encode())
        timeSend = time.time()
        while True:
            if self.ser.in_waiting > 0:
                ack = self.ser.readline().decode().strip()
                if ack == token:
                    print("{} received.".format(ack))
                    break
            if time.time() - timeSend > wait:
                print("Timeout, resending...")
                self.ser.write(dataCMD.encode())
                timeSend = time.time()
        self.isMoving = False
        # time.sleep(0.1)

    def startwalknew(self, wait = 1.5, token = "Forward"):
        dataCMD = json.dumps({'var':"move", 'val':1})
        self.ser.write(dataCMD.encode())
        timeSend = time.time()
        while True:
            if self.ser.in_waiting > 0:
                ack = self.ser.readline().decode().strip()
                if ack == token:
                    print("{} received.".format(ack))
                    break
            if time.time() - timeSend > wait:
                print("Timeout, resending...")
                self.ser.write(dataCMD.encode())
                timeSend = time.time()
        self.isMoving = False
        # time.sleep(0.1)

    def changeclearance(self, val = 20, wait = 1.5, token = "WALK_LIFT"):
        '''
        val: 0~30.
        '''
        dataCMD = json.dumps({'var': "ChangeClearance", 'val': val})
        self.ser.write(dataCMD.encode())
        timeSend = time.time()
        while True:
            if self.ser.in_waiting > 0:
                ack = self.ser.readline().decode().strip()
                if ack == token:
                    print("{} received.".format(ack))
                    break
            if time.time() - timeSend > wait:
                print("Timeout, resending...")
                self.ser.write(dataCMD.encode())
                timeSend = time.time()

    def readGlobalStep(self):
        value_str = self.ser.readline().decode().strip()
        # print("value_str:", value_str)
        global_step_boolean = re.search(r'Global_Step: (-?\d+\.\d+)', value_str)
        if global_step_boolean:
            self.globalStep = float(global_step_boolean.group(1))
        else:
            self.globalStep = -1.0
            print("WARNING: received string \"{}\" is not globalStep".format(value_str))

    def schedular(self):
        '''
        input: self.countCrossings, self.path
        output: action to take.
        '''
        if self.atCrossing and (self.prevCrossing == False) and (self.isMoving):
            self.countCrossing += 1

        return self.path[self.countCrossing]

    def rlbPID(self):
        '''
        input: 
            self.bottomLineYawStraight (the yaw along the forward direction, negative if the dog tilts to the left);
            self.rlbPIDParams (the Kp, Ki, Kd for the controller).
        output: self.RLB (float, the rlb value to the ESP32).
        return: NO RETURN VALUE.
        '''
        pass
        
    def buzzer(self, val):
        '''
        val: True (On) or False (Off)
        '''
        if val == True:
            dataCMD = json.dumps({'var': "buzzer", 'val': 1})
            self.ser.write(dataCMD.encode())
            print("Buzzer On")
        elif val == False:
            dataCMD = json.dumps({'var': "buzzer", 'val': 0})
            self.ser.write(dataCMD.encode())
            print("Buzzer Off")
        else:
            print("\nERROR: invalid logic value for the buzzer.\n")

    def buzzerPWM(self, val):
        dataCMD = json.dumps({'var': "buzzerPWM", 'val': val})
        self.ser.write(dataCMD.encode())

    def feetPosControl(self, positionMatrix):
        '''
        Controls 4 legs using '_inverseKinematics'.
        Input: positionMatrix should be a 4x3 matrix.
        '''
        for i in [1, 2, 3, 4]:
            self._inverseKinematics(i, *positionMatrix[i - 1])

    def _inverseKinematics(self, legNo, targetX, targetY, targetZ):
        '''
        The function for Inverse Kinematics for each leg. Four legs should be using this same function. Input legible values to prevent 
        value unsteadiness.
        Inputs: 1) serial number of the leg (No.1 to No.4); 2-4) target foot positions in the linkage model's coordinate frame;
        Outputs: None. It refreshes the linkageAngles internally when this function finishes.

        Step1: compute the angle offset for the Y-axis servo.
        Step2: compute the linkage angles for the 2 rear linkages (servoRear-A-C).
        Step3: compute the linkage angles for the 2 front linkages (servoFront-A-B).
        '''

        # ---- Step1 ---- #
        # from "void wigglePlaneIK()", computing in the Y-Z plane of the linkage frame.
        # this paragraph outputs:
        #  1) angleAlpha, which is the rotation angle of the whole leg around the X-axis of the linkage model frame;
        #  2) LB, i.e. the wiggle length, which is the foot's Y coordinate in the servo's coordinate frame, it differs from the targetY.
        if targetY > 0:
            L2C = targetZ ** 2 + targetY ** 2
            LC = np.sqrt(L2C)
            angleLambda = np.arctan2(targetZ, targetY) * self.E_PI  # convert from rad to deg.
            anglePsi = np.arccos(self.linkageW / LC) * self.E_PI  
            LB = np.sqrt(L2C - self.LWxLW)
            angleAlpha = anglePsi + angleLambda - 90
        elif targetY == 0:
            angleAlpha = np.arcsin(self.linkageW / targetZ) * self.E_PI  # ??? Why not arccos
            L2C = targetZ ** 2 + targetY ** 2
            LC = np.sqrt(L2C)  # modified from source code.
            LB = np.sqrt(LC - self.LWxLW)  # modified from source code.
        elif targetY < 0:
            targetY = - targetY
            L2C = targetZ ** 2 + targetY ** 2
            LC = np.sqrt(L2C)
            angleLambda = np.arctan2(targetZ, targetY) * self.E_PI
            anglePsi = np.arccos(self.linkageW / LC) * self.E_PI
            LB = np.sqrt(L2C - self.LWxLW)
            angleAlpha = 90 - angleLambda + anglePsi
        self.linkageAngles[legNo - 1][0] = angleAlpha
        self.wiggleLength[legNo - 1] = LB
        self.anglesOutput[legNo - 1][0] = angleAlpha

        # ---- Step2 ---- #
        # from "void singleLegPlaneIK()", computing in the X-Y plane of the leg frame.
        # this paragraph outputs:
        # 1) the rotation angle of the rear servo;
        # 2) the x and y position of the intersecting joint of linkageC and linkageB in the plane of the leg (frame origin is the midpoint between two servos).
        bufferS = np.sqrt((targetX + self.LSs2) ** 2 + LB ** 2)  # Euclidian distance from foot to the rear servo.
        print("bufferS {} | self.LAxLA {} | self.L_CD {} | self.LExLE {} | self.linkageA {} ".format(bufferS, self.LAxLA, self.L_CD, self.LExLE, self.linkageA))
        angleLambda = np.arccos((bufferS ** 2 + self.LAxLA - (self.L_CD + self.LExLE)) / (2 * bufferS * self.linkageA))  # the angle between bufferS and linkageA on the rear servo.
        angleDelta = np.arctan2((targetX + self.LSs2), LB)  # angle btw bufferS and wiggle length.
        angleBeta = angleLambda - angleDelta  # the rotation angle of the rear linkageA around the Z axis of the rear servo joint. (i.e. angle y-y')
        angleTheta = self.aLCDE  # a fixed angle opposing to the linkageC-linkageD line.
        angleOmega = np.arcsin((LB - np.cos(angleBeta) * self.linkageA) / self.sLEDC)  # an auxiliary angle.
        # print("angleOmega: {}".format(angleOmega))
        angleNu = np.pi - angleTheta - angleOmega  # an auxiliary angle between linkageE and the horizontal plane of foot.
        dEX = np.cos(angleNu) * self.linkageE  # projection length of linkageE to the X-axis.
        dEY = np.sin(angleNu) * self.linkageE  # projection length of linkageE to the Y-axis.
        # print("dEX, dEY: {} {}".format(dEX, dEY))
        angleMu = np.pi / 2 - angleNu  # angle btw linkageD and the horizontal plane of foot.
        dDX = np.cos(angleMu) * self.linkageD  # projection length of linkageD to the X-axis.
        dDY = np.sin(angleMu) * self.linkageD  # projection length of linkageD to the Y-axis.
        posBCX = targetX + dDX - dEX  # the X-coordinate of the intersection joint of linkageB and linkageC, in the plane of leg.
        posBCY = LB - dEY - dDY  # the Y-coordinate of the intersection joint of linkageB and linkageC, in the plane of leg.
        # print("posBCX, posBCY: {} {}".format(posBCX, posBCY))
        self.linkageAngles[legNo - 1][3] = angleBeta * self.E_PI  # update the leg's rear servo rotation angle.
        self.linkageAngles[legNo - 1][4] = -(np.pi - (np.pi / 2 - angleBeta) - angleOmega) * self.E_PI - self.angleEpsilon
        self.anglesOutput[legNo - 1][2] = angleBeta * self.E_PI

        # ---- Step3 ---- #
        # from "void simpleLinkageIK()", computing in the X-Y plane of the leg frame.
        # This paragraph outputs:
        # 1) angleAlpha: drives the servo;
        # 2) angleDelta: the rotation angle of front linkageA for the linkage model (not used for output);
        # 3) angleBeta: for the linkage model of linkageB.
        if posBCX - self.LSs2 == 0:  # if the joint is right below the origin.
            anglePsi = np.arccos((self.LAxLA_LBxLB + posBCY ** 2) / (self.LAx2 * posBCY)) * self.E_PI
            angleAlpha = 90 - anglePsi  # the offset of linkageA-front from the Y-axis.
            angleOmega = np.arccos((posBCY ** 2 + self.LBxLB_LAxLA) / (self.LBx2 * posBCY)) * self.E_PI  # the auxiliary angle opposing to front linkageA.
            angleBeta = anglePsi + angleOmega  # auxiliary angle between front linkageA and the extension of linkageB.
        else:
            L2C = posBCY ** 2 + (posBCX - self.LSs2) ** 2
            LC = np.sqrt(L2C)  # the Euclidian distance from the joint to the front servo.
            angleLambda = np.arctan2((posBCX - self.LSs2), posBCY) * self.E_PI  # WILL BE NEGATIVE when the joint is behind the front servo.
            anglePsi = np.arccos((self.LAxLA_LBxLB + L2C) / (2 * self.linkageA * LC)) * self.E_PI  # will always be positive.
            angleAlpha = 90 - angleLambda - anglePsi  # the offset of linkageA-front from the Y-axis.
            angleOmega = np.arccos((self.LBxLB_LAxLA + + L2C) / (2 * LC * self.linkageB)) * self.E_PI
            angleBeta = anglePsi + angleOmega
        angleDelta = 90 - angleAlpha - angleBeta  # the rotation angle of the front linkageA around the Z-axis of the front servo.
        self.linkageAngles[legNo - 1][1] = angleDelta 
        self.linkageAngles[legNo - 1][2] = angleBeta  # rotation of linkageB around jointA-B axis-Z.
        self.anglesOutput[legNo - 1][1] = angleAlpha

    def propagateAllLegJointPoses(self):
        '''
        This function calls the '_propagateSingleLegJointPoses' method to propagate all four leg's poses.
        '''
        for i in [1, 2, 3, 4]:
            self._propagateSingleLegJointPoses(i)

    def _propagateSingleLegJointPoses(self, legNo, verbose=False):
        '''
        This function calculates all the joint poses of a single leg in the ground coordinate frame.
        Propagation: Joint -> Previous Joint -> Servo Attached -> Origin of Linkage Frame -> Rigid Body -> Ground.
        Only to be called internally.
        NOTE: the linkage frame will be using right hand system for all four legs.
        '''
        
        if legNo == 1 or legNo == 2:  # for the left legs.
            # ---- the linkage frame origin -> ground ---- #
            linkageOriginPoseGround = hmRPYG(*self.bodyPose[:3], self.bodyPose[3:]).dot(hmRPYP(-90 + self.linkageAngles[legNo - 1][0], 0, 0, self.linkageFrameOffsets[legNo - 1]))
            # ---- zero position of the front / back servo -> ground ---- #
            frontServoZeroPoseGround = linkageOriginPoseGround.dot(hmRPYP(0, 0, 0, np.array([self.LSs2, 0, self.linkageW]).copy()))
            backServoZeroPoseGround = linkageOriginPoseGround.dot(hmRPYP(0, 0, 0, np.array([- self.LSs2, 0, self.linkageW]).copy()))
        elif legNo == 3 or legNo == 4:
            # ---- the linkage frame origin -> ground ---- #
            linkageOriginPoseGround = hmRPYG(*self.bodyPose[:3], self.bodyPose[3:]).dot(hmRPYP(-90 - self.linkageAngles[legNo - 1][0], 0, 0, self.linkageFrameOffsets[legNo - 1]))
            # ---- zero position of the front / back servo -> ground ---- #
            frontServoZeroPoseGround = linkageOriginPoseGround.dot(hmRPYP(0, 0, 0, np.array([self.LSs2, 0, - self.linkageW]).copy()))
            backServoZeroPoseGround = linkageOriginPoseGround.dot(hmRPYP(0, 0, 0, np.array([- self.LSs2, 0, - self.linkageW]).copy()))
        
        # ---- the front linkageA ---- #
        frontAPoseGround = frontServoZeroPoseGround.dot(hmRPYP(0, 0, self.linkageAngles[legNo - 1][1], np.array([0, 0, 0]).copy()))
        self.linkageCoordinatesGround[legNo - 1][0][:3] = R.from_matrix(frontAPoseGround[:3, :3]).as_euler('zyx', degrees = True)[::-1]
        self.linkageCoordinatesGround[legNo - 1][0][3:] = frontAPoseGround[:3, -1].flatten()
        # ---- the back linkageA ---- # 
        backAPoseGround = backServoZeroPoseGround.dot(hmRPYP(0, 0, self.linkageAngles[legNo - 1][3], np.array([0, 0, 0]).copy()))
        self.linkageCoordinatesGround[legNo - 1][2][:3] = R.from_matrix(backAPoseGround[:3, :3]).as_euler('zyx', degrees = True)[::-1]
        self.linkageCoordinatesGround[legNo - 1][2][3:] = backAPoseGround[:3, -1].flatten()
        # ---- the linkageB ---- #
        BPoseGround = frontAPoseGround.dot(hmRPYP(0, 0, self.linkageAngles[legNo - 1][2], np.array([0, self.linkageB, 0]).copy()))
        self.linkageCoordinatesGround[legNo - 1][1][:3] = R.from_matrix(BPoseGround[:3, :3]).as_euler('zyx', degrees = True)[::-1]
        self.linkageCoordinatesGround[legNo - 1][1][3:] = BPoseGround[:3, -1].flatten()
        # ---- the linkageC ---- #
        CPoseGround = backAPoseGround.dot(hmRPYP(0, 0, self.linkageAngles[legNo - 1][4], np.array([0, self.linkageC, 0]).copy()))
        if verbose:
            print("CPoseGround: {}".format(CPoseGround))
        self.linkageCoordinatesGround[legNo - 1][3][:3] = R.from_matrix(CPoseGround[:3, :3]).as_euler('zyx', degrees = True)[::-1]
        self.linkageCoordinatesGround[legNo - 1][3][3:] = CPoseGround[:3, -1].flatten()


    def drawAllLegLinkagesOG(self):
        '''
        This function calls the '_drawSingleLegLinkagesOG' method to draw four legs' linkages using OpenGL.
        '''
        for i in [1, 2, 3, 4]:
            self._drawSingleLegLinkagesOG(i, drawCoordinates = True)

    def _drawSingleLegLinkagesOG(self, legNo, drawCoordinates = True):
        """
        This function draws the linkages of a single leg using OpenGL.
        Only to be called internally.
        """
        # ---- front linkageA ---- #
        if drawCoordinates == True:
            hm = hmRPYP(*self.linkageCoordinatesGround[legNo - 1][0][:3], self.linkageCoordinatesGround[legNo - 1][0][3:])
            drawGroundOG(hm, scale=0.3)
        drawLineOG(self.linkageCoordinatesGround[legNo - 1][0][3:], self.linkageCoordinatesGround[legNo - 1][1][3:])

        # ---- linkageB ---- #
        hm = hmRPYP(*self.linkageCoordinatesGround[legNo - 1][1][:3], self.linkageCoordinatesGround[legNo - 1][1][3:])
        if drawCoordinates == True:
            drawGroundOG(hm, scale=0.3)
        posJointBC = hm.dot(np.array([0, self.linkageB, 0, 1]).T)[:3]
        drawLineOG(self.linkageCoordinatesGround[legNo - 1][1][3:], posJointBC)

        # ---- rear linkageA ---- #
        if drawCoordinates == True:
            hm = hmRPYP(*self.linkageCoordinatesGround[legNo - 1][2][:3], self.linkageCoordinatesGround[legNo - 1][2][3:])
            drawGroundOG(hm, scale=0.3)
        drawLineOG(self.linkageCoordinatesGround[legNo - 1][2][3:], self.linkageCoordinatesGround[legNo - 1][3][3:])

        # ---- linkageC, linkageD, LinkageE ---- #
        hm = hmRPYP(*self.linkageCoordinatesGround[legNo - 1][3][:3], self.linkageCoordinatesGround[legNo - 1][3][3:])
        if drawCoordinates == True:
            drawGroundOG(hm, scale=0.3)
        posJointDE = hm.dot(np.array([0, self.linkageC + self.linkageD, 0, 1]).T)[:3]
        posFoot = hm.dot(np.array([-self.linkageE, self.linkageC + self.linkageD, 0, 1]).T)[:3]
        drawLineOG(self.linkageCoordinatesGround[legNo - 1][3][3:], posJointDE)
        drawLineOG(posJointDE, posFoot)

    def drawRobotBody(self):
        self.body_verticesGround = hmRPYG(*self.bodyPose[:3], self.bodyPose[3:]).dot(self.body_vertices)
        drawRigidBodyOG(self.body_verticesGround)

    def forward(self, speed=50):
        dataCMD = json.dumps({'var':"move", 'val':1})
        self.ser.write(dataCMD.encode())
        print('robot-forward')
    
    def stopLR(self):
        dataCMD = json.dumps({'var':"move", 'val':6})
        self.ser.write(dataCMD.encode())
        print('robot-stop')

    def stopFB(self):
        dataCMD = json.dumps({'var':"move", 'val':3})
        self.ser.write(dataCMD.encode())
        print('robot-stop')

    def adjustHeight(self, height, dis = 1.0, token = "Adjusting Height", wait = 1.5): # TODO: fill the blanks.
        dataCMD = json.dumps({'var': "UPDOWN", 'val': height, 'dis': dis}) # TODO: fill the blanks.
        self.ser.write(dataCMD.encode())
        timeSend = time.time()
        while True:
            if self.ser.in_waiting > 0:
                ack = self.ser.readline().decode().strip()
                if ack == token:
                    print("{} received.".format(ack))
                    break
            if time.time() - timeSend > wait:
                print("Timeout, resending...")
                self.ser.write(dataCMD.encode())
                timeSend = time.time()
        print("Adjusted height to {}.".format(height))
        # change the walk clearance to default.
        # self.changeclearance()

    def poseUpdate(self):
        # TODO: implement EKF. For now, finish a demo to update pose based on pseudo control.
        self.initialEstimate = self.updatedEstimate

        # (optional) draw the pose of the center of robot.
        drawGround(self.hm(*self.updatedEstimate[:3], self.updatedEstimate[3:]), self.ax, "")

        # (optional) update the rigid body vertices.
        # self.body_verticesGround = self.hm(*self.updatedEstimate[:3], self.updatedEstimate[3:]).dot(self.body_vertices)
        # drawRigidBody(self.body_verticesGround, self.ax)

    def initBottomCamera(self):
        '''
        Initialize the bottom camera with cv2 video capturer.
        Output: the state of the camera capture, True/False.
        '''
        self.cap = cv2.VideoCapture("/dev/video"+str(self.bottomCamSrc))  # the index of the source camera.
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.bottomCamRes[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.bottomCamRes[1])
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # for the arducam config.
        # self.cap.set(cv2.CAP_PROP_FPS, 100)  # for the arducam config.
        print("Camera FPS is: {}".format(self.cap.get(cv2.CAP_PROP_FPS)))
        return self.cap.isOpened()

    def _detectSmallDots(self, frame, minDist = 13, minRadius = 4, maxRadius = 8, verbose = False):
        '''
        Detect the smaller dot patterns.
        Output: the center coordinates of the patterns and the distance Matrix.
        '''
        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply median blur to reduce noise
        gray_blurred = cv2.medianBlur(gray, 5)

        # Detect circles using HoughCircles
        circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=self.config['smallCirclesMinDistance'], param1=self.config['smallCirclesParam1'], param2=self.config['smallCirclesParam2'], minRadius=self.config['smallCirclesMinRadius'], maxRadius=self.config['smallCirclesMaxRadius'])  # param1=50, param2=30

        # Ensure at least one circle was found
        if circles is not None:
            circles = np.uint16(np.around(circles))
            count = 0
            centers = []

            for circle in circles[0, :]:
                count += 1
                center = (circle[0], circle[1])  # Circle center
                centers.append(center)
                if verbose: print("No {} circle's center: {}".format(count, center))

            centers = np.array(centers).reshape(-1, 2).astype('float')

            if count > 1:
                # get the mutual distance matrix.
                disMtx = distance.cdist(centers, centers, metric='euclidean')
                if verbose: print("disMtx: \n{}\n".format(disMtx))
                return centers, disMtx
            else:
                if verbose: print("Only {} circle detected, unable to calculate mutual distance.".format(count))
                return centers, None
            
        else:
             if verbose: print("No smaller dots in this frame.")
             return None
    
    # function to detect the Manhattan centroid.
    def _getManhattanCentroid(self, centers):
        '''
        Only use it when there is a crossing.
        input: an np ndarray of centers of circles.
        output: an 1x2 tuple of the manhattan centroid, which is one of the circle centers.
        '''
        # compute the mutual distance.
        disMtx = distance.cdist(centers, centers, metric='euclidean')
        # find the Manhattan centroid.
        sumEucDis = np.sum(disMtx, axis=0)
        mcIdx = np.argmin(sumEucDis)
        mc = centers[mcIdx, :]
        return mc
    
    def _groupCircles(self, centers, disMtx, disThres = 45.0, verbose = False):
        '''
        Detect the smaller dot patterns to get the relative pose of the next brick.
        Amount of input centers should exceed 1.
        '''
        disMtxVisited = np.eye(disMtx.shape[0]).astype('bool')
        group1 = np.array([])
        group2 = np.array([])
        for row in range(disMtx.shape[0]):
            for column in range(disMtx.shape[1]):
                if disMtxVisited[row, column] == True:
                    continue
                else:
                    disMtxVisited[row, column] = True
                    disMtxVisited[column, row] = True
                    if disMtx[row, column] >= disThres:
                        if row in group1:
                            if column in group2:
                                continue
                            else:
                                group2 = np.append(group2, column)
                        elif row in group2:
                            if column in group1:
                                continue
                            else:
                                group1 = np.append(group1, column)
                        else:
                            if column in group1:
                                group2 = np.append(group2, row)
                            elif column in group2:
                                group1 = np.append(group1, row)
                            else:
                                group1 = np.append(group1, row)
                                group2 = np.append(group2, column)
                    else:
                        if row in group1:
                            if column in group1:
                                continue
                            else:
                                group1 = np.append(group1, column)
                        elif row in group2:
                            if column in group2:
                                continue
                            else:
                                group2 = np.append(group2, column)
                        else:
                            if column in group1:
                                group1 = np.append(group1, row)
                            elif column in group2:
                                group2 = np.append(group2, row)
                            else:
                                group1 = np.append(group1, row)
                                group1 = np.append(group1, column)
        '''
        if len(np.intersect1d(group1, group2)) > 0:
            group1 = np.union1d(group1, group2)
            group2 = np.array([])
        '''
        if verbose: print("Group1: {} | Group2: {}\n".format(group1, group2))
        comGroup1 = np.mean(centers[group1.astype('int')], axis=0)
        comGroup2 = np.mean(centers[group2.astype('int')], axis=0)
        if comGroup1[0] <= comGroup2[0]:  # related to the orientation of the camera.
            self.previousCircles = len(group1)
            self.nextCircles = len(group2)
        else:
            self.previousCircles = len(group2)
            self.nextCircles = len(group1)
        


    def _detectCircles(self, frame, minCircles = 3, savetomat = False, verbose = False, display = False):
        
        '''
        Input: the cv2 captured frame (should be 120x160x3);
        Output: a list containing the centers and the mutual yaw matrix. None if no circle in the input frame.
        '''

        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply median blur to reduce noise
        gray_blurred = cv2.medianBlur(gray, 5)

        # Detect circles using HoughCircles
        # params can be tuned if necessary.
        # circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=10, param1=50, param2=21, minRadius=2, maxRadius=10)  # default: param1=50, param2=30
        circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=self.config['largeCirclesMinDistance'], param1=self.config['largeCirclesParam1'], param2=self.config['largeCirclesParam2'], minRadius=self.config['largeCirclesMinRadius'], maxRadius=self.config['largeCirclesMaxRadius'])

        # Ensure at least one circle was found
        if circles is not None:
            circles = np.uint16(np.around(circles))
            count = 0
            centers = []
            # iterate over each detected circle.
            for circle in circles[0, :]:
                count += 1
                center = (circle[0], circle[1])  # Circle center
                centers.append(center)
                if verbose: print("No {} circle's center: {}".format(count, center))
                radius = circle[2]  # Circle radius
                if display: 
                    # Draw a small circle at the center
                    cv2.circle(frame, center, 2, (0, 255, 0), 1)
                    # Draw the detected circle
                    cv2.circle(frame, center, radius, (0, 0, 255), 1)
                    # Put text.
                    cv2.putText(frame, str(count), (center[0] - 2, center[1] - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
            centers = np.array(centers).reshape(-1, 2).astype('float')
            # find the Manhattan centroid.
            # centroid = self._getManhattanCentroid(centers)
            # centroid = np.mean(centers, axis = 0)

            # init yaw storage.
            yaw = []

            if count >= minCircles:
                # calculate the mutual gradient.
                x = centers[:, 0]
                y = centers[:, 1]
                xx = x[:, np.newaxis]
                yy = y[:, np.newaxis]
                dx = xx - x
                dy = yy - y
                # yaw = np.arctan(np.divide(dy, dx).copy()) * 180 / np.pi
                yaw = np.arctan(np.divide(dx, dy).copy()) * 180 / np.pi  # depending on the orientation of the camera.
                if verbose: print("Mutual gradient of {} circles: \n{}".format(count, yaw))
                # save to mat file.
                if savetomat: savemat('zh_CircleOutput.mat', {"yaw": yaw})
            else:
                if verbose: print("Only {} circle detected, unable to calculate gradient.")

            if display:
                # Display the processed frame
                cv2.imshow('Detected Circles', frame)       
            return [centers, yaw]
        
        else:
            if verbose: print("No circle in this frame.")
            if display:
                # Display the processed frame
                cv2.imshow('Detected Circles', frame)  
            return None

    def _computeYawFromMutualGradients(self, yaw, minCircles = 3, tolerance = 8, verbose = False):
        '''
        compute the 2D pose of the robot body based on the mutual yaw matrix returned by the 
        circles.

        Input: mutual yaw mtx, in degrees.
        Output: an array containing 1 yaw (if only one line clustered), 
                or 2 if two lines are clustered.
        '''
        lineYaw = np.array([])
        entryVisited = np.zeros((yaw.shape)).astype('bool')  # boolean matrix to register which entry has been visited.
        entryDir = np.zeros((yaw.shape[0])).astype('bool')
        for row in range(yaw.shape[0]):
            for column in range(yaw.shape[1]):
                if entryVisited[row][column] == False and entryVisited[column][row] == False:
                    idx = np.where(np.abs(yaw[row, :] - yaw[row, column]) <= tolerance)[0]  # search circles in the same row with a similar mutual yaw.
                    if verbose: print("idx:\n{}\n".format(idx))
                    if len(idx) >= minCircles:  # too few circles may result in noise.
                            meanYaw = np.mean(yaw[row, idx])
                            groupLine = np.where(np.abs(lineYaw - meanYaw) <= tolerance)[0]
                            # if the registered group does not contain this yaw angle, 
                            # add this yaw to the registered group.
                            if groupLine.size == 0:  
                                if np.abs(meanYaw) <= 45:  # mind the direction of the camera.
                                    entryDir[idx] = True
                                lineYaw = np.append(lineYaw, meanYaw)
                            else:
                                if np.abs(meanYaw) <= 45:
                                    entryDir[idx] = True
                                # update the mean yaw if this line already exists in the registered group.
                                lineYaw[groupLine] = (lineYaw[groupLine] + meanYaw) / 2  
                            # update the visited circles, including the symmetrical ones.
                            entryVisited[row, idx] = True
                            entryVisited[idx, row] = True
                            if verbose:
                                print("Line(s) found.\n")
                                print("mean:\n{}\n".format(meanYaw))
                                print("groupLine:\n{}\n".format(groupLine))
                                print("lineYaw:\n{}\n".format(lineYaw))
                                print("entryVisited:\n{}\n".format(entryVisited))
                    else:
                        if verbose: print("No lines found.\n")
                        entryVisited[row, idx] = True
                        entryVisited[idx, row] = True
                    if verbose: print("----------------------------------------------------------")
        # If more than 2 lines were clustered, this result should be dumped;
        # if 2 lines were clustered but they are not orthogonal, this result should also be dumped.
        # print("lineYaw: {}".format(lineYaw))
        if lineYaw.size > 2: lineYaw = np.array([False])
        elif lineYaw.size == 2 and np.abs(np.abs(lineYaw[0] - lineYaw[1]) - 90) > tolerance: 
            if verbose: print("lineYaw.size == 2")
            lineYaw = np.array([False])
        elif lineYaw.size == 0: lineYaw = np.array([False])
        return np.sort(lineYaw), entryDir


    def _enqueue(self, item):
        '''
        General purpose function to pop-in new items to a FIFO queue. If the queue is full, the earliest item will be removed.
        '''
        self.atCrossingFIFO.append(item)
        
    def _viewQueue(queue):
        '''
        Check all the items in the queue, in a list.
        '''
        return list(queue)

    def checkCrossing(self):
        '''
        Check whether there is a crossing based on the FIFO.
        '''
        listFIFO = list(self.atCrossingFIFO)
        if sum(listFIFO) < len(listFIFO) // 2:
            self.prevCrossing = self.atCrossing
            self.atCrossing = False
        elif sum(listFIFO) >= len(listFIFO) // 2:
            self.prevCrossing = self.atCrossing
            self.atCrossing = True

    def getPoseFromCircles(self, minCircles = 5, verbose=False, display=False):
        '''
        To detect the circle patterns on the bricks for EACH FRAME.
        Input: config params (minCircles means the frame will be dumped if there are 
                less than this amount of circles).
        Output: the centers of the detected circles. None if no frame is 
                returned by the camera or no lines detected in the frame.
        '''
        ret, frame = self.cap.read()
        if not ret:
            print("ERROR: Camera not returning values.")
            return None
        
        ret1 = self._detectCircles(frame.copy(), minCircles=minCircles, display=display)
        retSmall = self._detectSmallDots(frame.copy(), verbose=verbose)

        # process the large circle traces.
        if ret1 != None:
            bottomLineYaw, self.entryDir = self._computeYawFromMutualGradients(np.array(ret1[1]), minCircles=minCircles, verbose=verbose)
            # print("bottomLineYaw: {}".format(bottomLineYaw))
            # compute the straight line manhattan centroid
            if self.entryDir.any():
                bottomLineCentroid = self._getManhattanCentroid(ret1[0][self.entryDir])
                self.bottomLineCentroid = bottomLineCentroid - np.array(self.bottomCamRes) / 2
                self.bottomLineCentroid = (self.bottomLineCentroid + bottomLineCentroid) / 2  # windowing.
                self.walkDir = - 2 * np.arctan2(self.bottomLineCentroid[0], self.denoConst) * 180 / np.pi  # pay attention to the camera's orientation.
            if bottomLineYaw.all():  # only change the yaw when the detected yaw(s) is legitimate.
                # self.bottomLineYaw = bottomLineYaw
                bottomLineYaw *= -1  # the actual direction
                self.bottomLineYaw = bottomLineYaw
                # determine if the robot detects a crossing.
                if len(self.bottomLineYaw) == 2: 
                    self._enqueue(True)
                    # select one yaw from the two detected value.
                    # bottomLineYaw = bottomLineYaw[np.argmin(np.abs(bottomLineYaw))]
                    # bottomLineYaw = np.sign(bottomLineYaw) * np.abs(bottomLineYaw)
                    # self.bottomLineYawStraight = bottomLineYaw
                    self.bottomLineYawStraight = self.bottomLineYaw[np.argmin(np.abs(bottomLineYaw))]
                elif len(self.bottomLineYaw) == 1:
                    if abs(self.bottomLineYaw) >= 50:
                        # a vertical line detected.
                        self._enqueue(True)
                    else:
                        if self.entryDir.any():
                            self.bottomLineYawStraight = self.bottomLineYaw[0]
                        self._enqueue(False)
                # prevent datatype errors.
                if isinstance(self.bottomLineYawStraight, (int, float)) != True:
                        self.bottomLineYawStraight = self.bottomLineYawStraight[0]
                # check if there is a crossing, with a time window.
                self.checkCrossing()
                # determine the current action to take.
                self.currentAction = self.schedular()
                # check whether the traces are visible to the robot.
                if np.abs(self.bottomLineCentroid[0]) < self.horizontalLimit:
                    self.lostVision = 0
                else:
                    if self.bottomLineCentroid[0] <= 0:
                        self.lostVision = -2
                    else:
                        self.lostVision = 2
            # else:
            if (not bottomLineYaw.all()) or (not self.entryDir.any()):
                # print("bottomLineYaw: {}, self.entryDir: {}".format(bottomLineYaw, entryDir))
                if self.bottomLineCentroid[0] < 0:
                    self.lostVision = -1
                elif self.bottomLineCentroid[0] > 0:
                    self.lostVision = 1
            if verbose: print("Manhattan Centroid: [{:.2f}, {:.2f}] | walkDir: {} | yaw(s): {}\n".format(self.bottomLineCentroid[0], self.bottomLineCentroid[1], self.walkDir, self.bottomLineYaw))
            # return [self.bottomLineCentroid, self.bottomLineYaw]
        else:
            print("ret1 is {}.".format(ret1))

        # process the small dot patterns.
        if retSmall != None:
            # print("retSmall[1]: ", retSmall[1])
            if retSmall[1] is None:
                # only one dot detected, either up or down the short side of the brick.
                # TODO: reserve a stair detection API. The following is just temporary.
                self.previousCircles = 1
                self.nextCircles = 0
            else:
                # more the 1 dot detected.
                self._groupCircles(retSmall[0], retSmall[1])
        # else:
            # if nothing in the current frame return the previous result.
            # return [self.bottomLineCentroid, self.bottomLineYaw]


    def measurementUpdate(self, results, useCalibration = True):
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
            eulerCamera = rotCamera.as_euler('zyx', degrees = True)  # it uses right hand coordinate system.
            transCamera = homo[:-1, -1].flatten()
            transCamera[0] *= -1
            # Add the optional calibration function.
            if useCalibration:
                transCamera = calibratePose2D(eulerCamera[1], transCamera)[1]
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
        
# width of 3 tags without edge: 56.6cm, interval: 2.2cm. width of tags: 17.2cm
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
                 [11, 90, 180, 0, -(0.172 + 0.022), -0.35, 0.055 + 0.172/2]
                 ])

# adjust the biases of tags.
poseTags[0:3, 4] += boardBiasesX[0]
poseTags[3:6, 5] += boardBiasesX[1]
poseTags[6:9, 4] += boardBiasesX[2]
poseTags[9:12, 5] += boardBiasesX[3]

poseTags[0:3, 5] += boardBiasesY[0]
poseTags[3:6, 4] += boardBiasesY[1]
poseTags[6:9, 5] += boardBiasesY[2]
poseTags[9:12, 4] += boardBiasesY[3]

trackInner = np.array([[0.1, 0.694, 0.694, 0.1, 0.1], 
                       [0.1, 0.1, 0.695, 0.695, 0.1],
                       [0, 0, 0, 0, 0]])
trackOuter = np.array([[-0.1, 0.894, 0.894, -0.1, -0.1],
                       [-0.1, -0.1, 0.895, 0.895, -0.1],
                       [0, 0, 0, 0, 0]])

# a simpler trajectory, in the world frame.
trajectory = np.array([[0.0, 0.994 - 0.20, 0.994 - 0.20, 0.0, 0.0],  # x coordinate
                       [0.0, 0.0, 0.995 - 0.20, 0.995 - 0.20, 0.0],  # y coordinate
                       [0.0, 0.0, 0.0, 0.0, 0.0]])  # z coordinate

# tag grouping, each row represents the tagIDs on the same board.

tagGroups = np.array([[3, 4, 5],
                      [1, 7, 6],
                      [8, 9, 0],
                      [2, 10, 11]])

'''
tagGroups = np.array([[15, 16, 17]])  # tag 15~17 are only for DEMO USE.
'''

# function of Yaw and X, Z calibration.
def calibratePose2D(yaw, trans):
    # Calibrate Yaw:
    biasAngle = np.arcsin(trans[0] / trans[-1])
    biasAngle = np.degrees(biasAngle)
    yaw = yaw + biasAngle
    yawRad = np.radians(yaw)

    # calibrate X translation.
    trans[0] = trans[-1] * np.sin(yawRad)
    trans[-1] = trans[-1] * np.cos(yawRad)

    return yaw, trans

def checkTurning(brickNo, pose, trajectory):
    corner = trajectory[:, brickNo + 1]
    yaw = pose[1]
    targetVector = corner - pose[3:]
    targetVector = hmRPYP(0, 0, -90*brickNo, np.array([0, 0, 0]))[:3, :3].dot(targetVector.reshape(-1, 1))[:-1]
    return yaw, targetVector

def drawBrick(pose, vertices, ax):
    x, y, z = hmRPYG(*pose[:3], pose[3:]).dot(vertices)[:3, :]
    # print("x = {}, y = {}, z = {}:".format(x, y, z))

    xx = np.linspace(np.min(x), np.max(x), 100)
    yy = np.linspace(np.min(y), np.max(y), 100)
    zz = np.linspace(np.min(z), np.max(z), 10)
 
    # Parallel to the X-Y plane
    xxGrid, yyGrid = np.meshgrid(xx, yy)
    zzGrid = np.full_like(xxGrid, np.max(z))
    ax.plot_surface(xxGrid, yyGrid, zzGrid, color = 'lightblue', alpha = 1, rasterized = True)
    zzGrid = np.full_like(xxGrid, np.min(z))
    ax.plot_surface(xxGrid, yyGrid, zzGrid, color = 'lightblue', alpha = 1, rasterized = True)

    # Parallel to X-Z plane
    xxGrid, zzGrid = np.meshgrid(xx, zz)
    yyGrid = np.full_like(xxGrid, np.max(y))
    ax.plot_surface(xxGrid, yyGrid, zzGrid, color = 'navy', alpha = 1, rasterized = True)
    yyGrid = np.full_like(xxGrid, np.min(y))
    ax.plot_surface(xxGrid, yyGrid, zzGrid, color = 'navy', alpha = 1, rasterized = True)

    # Parallel to Y-Z plane
    yyGrid, zzGrid = np.meshgrid(yy, zz)
    xxGrid = np.full_like(yyGrid, np.max(x))
    ax.plot_surface(xxGrid, yyGrid, zzGrid, color = 'navy', alpha = 1, rasterized = True)
    xxGrid = np.full_like(yyGrid, np.min(x))
    ax.plot_surface(xxGrid, yyGrid, zzGrid, color = 'navy', alpha = 1, rasterized = True)

def drawBrickOG(pose, vertices):
    verticesNew = hmRPYG(*pose[:3], pose[3:]).dot(vertices)[:3, :].T # 8x3 matrix
    glLineWidth(5.0)
    glBegin(GL_QUADS)
    # draw the top and bottom surfaces
    for i in [0, 1, 2, 3]:
        glColor3fv((0.2, 0.5, 0.35))
        glVertex3dv(verticesNew[i])
    for i in [4, 5, 6, 7]:
        glColor3fv((0.2, 0.5, 0.35))
        glVertex3dv(verticesNew[i])
    # draw the sides
    for i in [0, 1, 5, 4]:
        glColor3fv((0.1, 0.25, 0.1))
        glVertex3dv(verticesNew[i])
    for i in [1, 2, 6, 5]:
        glColor3fv((0.1, 0.25, 0.1))
        glVertex3dv(verticesNew[i])
    for i in [2, 3, 7, 6]:
        glColor3fv((0.1, 0.25, 0.1))
        glVertex3dv(verticesNew[i])
    for i in [3, 0, 4, 7]:
        glColor3fv((0.1, 0.25, 0.1))
        glVertex3dv(verticesNew[i])
    glEnd()

class brickMap():
    def __init__(self, hm, ax) -> None:
        '''
        Init the class to store brick model and update the brick map.
        '''
        self.hm = hm
        # self.ax = ax
        self.brickLength = 0.40 # meter
        self.brickWidth = 0.20
        self.brickThickness = 0.015
        self.brickVertices = np.array([[0.5 * self.brickLength, 0.5 * self.brickLength, - 0.5 * self.brickLength, - 0.5 * self.brickLength, 
                                        0.5 * self.brickLength, 0.5 * self.brickLength, - 0.5 * self.brickLength, - 0.5 * self.brickLength],
                                       [0.5 * self.brickWidth, - 0.5 * self.brickWidth, - 0.5 * self.brickWidth, 0.5 * self.brickWidth, 
                                        0.5 * self.brickWidth, - 0.5 * self.brickWidth, - 0.5 * self.brickWidth, 0.5 * self.brickWidth],
                                       [0.5 * self.brickThickness, 0.5 * self.brickThickness, 0.5 * self.brickThickness, 0.5 * self.brickThickness, 
                                        - 0.5 * self.brickThickness, - 0.5 * self.brickThickness, - 0.5 * self.brickThickness, - 0.5 * self.brickThickness],
                                       [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]])  # padding for computation.
        # self.map = np.zeros(6).astype('float')  # to store the poses of the bricks as maps.
        self.map = np.array([])  # to utilize the append attribute of lists. len = No. of bricks.
        print("brickMap Initialized!")

    def place(self, pose):  # pose is a list len = 6.
        # TODO: detect collision and layer.
        if self._viabilityDetect(pose) == True:  # if the pose of brick is viable.
            self.map = np.append(self.map, pose).reshape(-1, 6)
            # drawGround(hmRPYG(*pose[:3], pose[3:]), self.ax, "Brick ".join(str(len(self.map + 1))))  # len(list) returns the rows of a list (first dim).
            '''
            drawRigidBody(hmRPYG(*pose[:3], pose[3:]).dot(self.brickVertices), self.ax)
            drawBrick(pose, self.brickVertices, self.ax)
            '''
            drawRigidBodyOG(hmRPYG(*pose[:3], pose[3:]).dot(self.brickVertices))
            drawBrickOG(pose, self.brickVertices)
            return True  #  to be passed to the robot class.
        else:
            print("Invalid Brick Pose")
            return False  #  to be passed to the robot class.

    def _viabilityDetect(self, pose) -> bool:  # pose is a list len = 6.
        # TODO: the rule check function before placing bricks.
        return True
    
# WiFi data TX function (for RPi).
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

# WiFi data RX function (for PC)
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

def keyboardCtrl(angleStep = 1, zoom_sensitivity = 0.001):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                glTranslatef(0.025, 0, 0)
            if event.key == pygame.K_RIGHT:
                glTranslatef(-0.025, 0, 0)
            if event.key == pygame.K_UP:
                glTranslatef(0, -0.025, 0)
            if event.key == pygame.K_DOWN:
                glTranslatef(0, 0.025, 0)
        if event.type == MOUSEBUTTONDOWN:
            if event.button == 1:
                glRotatef(angleStep, 1, 0, 0)
            elif event.button == 3:
                glRotatef(angleStep, 0, 0, 1)
            elif event.button == 4:  # wheel up.
                glTranslatef(0.0, 0.0, zoom_sensitivity)
            elif event.button == 5: 
                glTranslatef(0.0, 0.0, -zoom_sensitivity)

def drawFloor():
    vertices = np.array([[-0.4, -0.4, -0.02],
                         [1.5, -0.4, -0.02],
                         [1.5, 1.5, -0.02],
                         [-0.4, 1.5, -0.02]])
    glBegin(GL_QUADS)
    glColor3fv((0.85, 0.85, 0.85))
    for vertex in vertices:
        glVertex3fv(vertex)
    glEnd()
