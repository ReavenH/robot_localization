# define the ground coordinates:
# X: pointing forwards; Y: pointing left; Z: pointing upwards
# each column: x, y, z point coords,
import numpy as np
import matplotlib.pyplot as plt

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
    plt.pause(0.02)  # default 0.02


def drawRigidBody(vertices, ax):
    # vertices is the 8 vertices of the robot rigid body.
    links = [[0, 1], [1, 2], [2, 3], [3, 0],
            [4, 5], [5, 6], [6, 7], [7, 4],
            [0, 4], [1, 5], [2, 6], [3, 7]]
    vertices = vertices[:3, :].T  # should be 8x3 if the rigid body is a rectangular prism.
    for link in links:
        ax.plot3D(*zip(*vertices[link]), color="k", linewidth = 0.3)

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

trackInner = np.array([[0.1, 0.694, 0.694, 0.1, 0.1], 
                       [0.1, 0.1, 0.695, 0.695, 0.1],
                       [0, 0, 0, 0, 0]])
trackOuter = np.array([[-0.1, 0.894, 0.894, -0.1, -0.1],
                       [-0.1, -0.1, 0.895, 0.895, -0.1],
                       [0, 0, 0, 0, 0]])

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