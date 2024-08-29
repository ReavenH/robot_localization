#!usr/bin/python

'''
Implement EKF/KF to do data fusion.
'''
import numpy as np

class planarKF():
    '''
    A, B, C matrices are considered as identity matrices.
    '''
    def __init__(self) -> None:
        self.A = np.identity(6) # should be 6x6
        self.B = np.identity(6)
        self.C = np.identity(6)
        # TODO: measure the noise.
        self.Sigma = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(1, -1)  # should be 1x6
        self.Q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(1, -1)

    def step(self, muBar, sigmaBar, odometry, measurement):  # input from the robot class. mu 6x1; sigmaBar 6x6 covariance mtx.
        muBar = np.dot(self.A, muBar) + np.dot(self.B, odometry)  # odometry should be 6x1
        sigmaBar = self.A.dot(sigmaBar).dot(self.A.T)
        K = sigmaBar.dot(self.C.T).dot(np.linalg.inv(self.C.dot(sigmaBar).dot(self.C.T) + self.Q))
        mu = muBar + K.dot(measurement - self.C.dot(muBar))
        sigma = (np.identity(6) - K.dot(self.C)).dot(sigmaBar)
        return mu, sigma