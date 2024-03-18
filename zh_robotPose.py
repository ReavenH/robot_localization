import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


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


def drawRigidBody(vertices):
    # vertices is the 8 vertices of the robot rigid body.
    links = [[0, 1], [1, 2], [2, 3], [3, 0],
            [4, 5], [5, 6], [6, 7], [7, 4],
            [0, 4], [1, 5], [2, 6], [3, 7]]
    vertices = vertices[:3, :].T  # should be 8x3 if the rigid body is a rectangular prism.
    for link in links:
        ax_fig0.plot3D(*zip(*vertices[link]), color="k", linewidth = 0.3)


class robot():
    def __init__(self, hm, ax): # hm is the 4x4 homogeneous matrix, for different rotation orders.
        self.body_length = 0.20  # cm
        self.body_width = 0.10  # cm
        self.body_thickness = 0.03  # cm
        # the centroid of the robot, also the origin of the rigid body frame.
        # Can be adjusted to comply with the actual turning center.
        self.center = np.array([0.0, 0.0, 0.0])
        # to store the vertices of the robot body as a rectangular prism.
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
        self.measurement = np.zeros((4, 4)).astype('float')  # init the measurement of each update step.
        self.control = np.zeros(2).astype('float')  # init the 2D control input: translation along the X+ axis, delta-yaw.
        self.odometry = np.zeros(2).astype('float') # init the 2D odometry based on IMU.
        self.hm = hm  # pass the homogeneous matrix calculation func.
        self.initialEstimate = np.zeros(6).astype('float')  # init the initail estimate of pose: roll, pitch, yaw, x, y, z.
        self.updatedEstimate = np.zeros(6).astype('float')
        self.ax = ax  # the plot axis.

    def poseUpdate(self):
        # TODO: implement EKF. For now, finish a demo to update pose based on pseudo control.
        self.updatedEstimate = self.initialEstimate

        # draw the pose of the center of robot.
        drawGround(self.hm(*self.updatedEstimate[:3], self.updatedEstimate[3:]), self.ax, "Dog")
        # update the rigid body vertices.
        self.body_verticesGround = self.hm(*self.updatedEstimate[:3], self.updatedEstimate[3:]).dot(self.body_vertices)
        drawRigidBody(self.body_verticesGround)

    def perceptionUpdate(self, odo, mea):
        # TODO: transform the visual measurement from camera frame to the rigid body frame.
        self.odometry = odo
        self.measurement = mea

    def controlUpdate(self, ctrl):
        self.control = ctrl
        # update the init guess of pose (ground truth).
        # consider move the Gaussian noise here (forward noise).
        translation = np.array([[self.control[0], 0.0, 0.0, 1]]).T  # in the rigid body frame
        self.initialEstimate[3:] = self.hm(*self.updatedEstimate[:3], self.updatedEstimate[3:]).dot(translation).flatten()[:-1]  # x, y, z in the world frame.
        self.initialEstimate[2] = self.updatedEstimate[2] + self.control[1]  # only change yaw.


class landmarks():
    def __init__(self, hm, poses, ax):
        '''
        Init the ground truth locations of the Apriltags.
        TODO: How to specify the initial location when the robot class is instanciated? Relative position.
        '''
        self.hm = hm  # specify the type of homegeneous matrix.
        self.poses = poses  # poses corresponding to the homogeneous matrix. Shape: nx7, 7 = tagID + 6 poses, n = No. of tags.
        self.ax = ax  # drawing axis.


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
        self.map = np.zeros(6).astype('float')  # to store the poses of the bricks as maps.

    def place(self, pose):
        # TODO: detect collision and layer.
        pass


if __name__ == "__main__":
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
    poseTags = np.array([[13, 90, -90, 0, 0.75, 0.20, 0.13],
                         [14, 90, -90, 0, 0.75, -0.20, 0.13],
                         [15, 90, -90, 0, 0.75, -0.40, 0.13],
                         [16, 90, -90, 0, 0.75, 0.40, 0.13]])
    myTags = landmarks(hmRPYP, poseTags, ax_fig0)

    for _, pose in enumerate(myTags.poses):
            drawGround(myTags.hm(*pose[1:4], pose[4:]), myTags.ax, "Tag "+str(int(pose[0])))

    # ---- Define a pseudo control input series ----
    '''
    controls = np.array([[0.4, 0.4, 0.4, 0.4],
                         [90, 90, 90, 90]])
    '''
    controls = np.array([[0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                         [10, -10, -5, -5, 2, -2, 6, -6, 7]])
    
    controls[0] = controls[0] + np.random.normal(0, 0.02, 9)
    controls[1] = controls[1] + np.random.normal(0, 1, 9)

    # ---- Instantiate the robot class ----
    myRobot = robot(hmRPYG, ax_fig0)
    poseRecord = np.zeros((1, 6))
    for i, ctrl in enumerate(controls.T):
        myRobot.controlUpdate(ctrl)
        myRobot.poseUpdate()
        np.vstack((poseRecord, myRobot.updatedEstimate))
        
input("Press any key to close the figure...")
plt.close()
