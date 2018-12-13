#!/usr/bin/env python

"""
    @file kalman_filter_nd.py
    @brief Multi-dimensional Kalman Filter Class

    @author Connor Novak
    @author Charlie Weiss
"""

from collections import namedtuple
from Gaussian import Gaussian
import numpy as np
from scipy.stats import multivariate_normal

# Pseudocode
class KalmanFilterND():

    def __init__(self, prior, command=0.0):
        '''
            @brief init kf with gaussian

            @param[in] prior state tuple (mean vector, covariance matrix)
            @param[in] initial robot command (default to 0.0 m/s)
        '''

        # init prior w/ mean vector, covariance matrix from prior tuple
        self.prior = multivariate_normal(prior[0], prior[1])
        print(self.prior.pdf([2.5, 7.3]))

        self.dt = 0.5                       # Initial Timestep
        self.x = np.array([[0, 0]]).T
        self.P = np.array([[0, 500],
                           [500, 0]])       # Initial Covariance

        self.sigma_v = 0.2                  # Velocity variance (0.15<var<0.3)
        self.R = np.array([[0.5]])          # Measurement Noise Matrix
        self.Q = np.array([[0, 0              ],
                           [0, self.sigma_v**2]])   # Process Noise Matrix
        self.F = np.array([[1, 0],
                           [0, 0]])         # State Transition Function
        self.u = np.array([[command]])      # Control Input
        self.B = np.array([[self.dt, 1]]).T # Control Function
        self.H = np.array([[1, 0]])         # Measurement Function

        self.display()
        self.predict(self.u)

    def predict(self, u):
        '''
            @brief generate predicted state from prior, cmd_vel, noise

            @param[out] update state attribute x
            @param[out] update state covariance P
        '''
        self.x = self.F * self.x + self.B * self.u
        self.P = self.P + self.Q    # Combine covariances, decrease certainty

    def update(self,z):
        '''
            @brief generate estimated state from measurement, prior

            @param[out] update state attribute x
            @param[out] update state covariance P
        '''

        S = self.H * self.P * self.H.T + self.R     # System Uncertainty S
        y = z - self.H * self.x                     #
        K = self.P * self.H.T * np.linalg.inv(S)    # Kalman Gain K

    def step(self, z, dx, R=.001, Q=.001):
        measurement = Gaussian(z,R)
        movement = Gaussian(dx,Q)

        new_prior = self.predict(self.prior, movement)
        posterior = self.update(new_prior,measurement)
        self.prior = posterior
        return posterior.mean, posterior.variance

    def display(self):
        '''
            @brief prints settings and state of the filter
        '''

        print("Static Filter Settings")
        print("State Transition Function F:")
        print(self.F)

        print("Velocity Variance sigma_v:")
        print(self.sigma_v)

        print("Process Noise Matrix Q:")
        print(self.Q)

        print("Control Function B:")
        print(self.B)

        print("Control Input u:")
        print(self.u)

if __name__ == '__main__':
    mu = np.array([2.0, 7.0])
    P = np.array([[8., 0.],
                     [0., 3.]])
    print("mean vector: ")
    print(mu)
    print("covariance matrix: ")
    print(P)
    mdkf = KalmanFilterND((mu, P))
