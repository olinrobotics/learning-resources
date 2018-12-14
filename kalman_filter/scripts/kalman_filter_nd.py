#!/usr/bin/env python

"""
    @file kalman_filter_nd.py
    @brief Multi-dimensional Kalman Filter Class

    Runs display of full loop given hardcoded values in main function if called
    in isolation. Built to be instantiated and run as part of a larger script.

    TODO(connor): Add flags for prediction vs update for error checking
    TODO(connor): make predict, update "private"
    TODO(connor): Add error checking for numpy arrays, proper dimensions

    @author Connor Novak
    @author Charlie Weiss
"""

from collections import namedtuple
from Gaussian import Gaussian
import numpy as np
from scipy.stats import multivariate_normal

# Pseudocode
class KalmanFilterND():

    def __init__(self, prior):
        '''
            @brief init kf with gaussian

            @param[in] prior state tuple (mean vector, covariance matrix)
            @param[in] initial robot command (default to 0.0 m/s)
        '''

        self.dt = 0.5                       # Initial Timestep

        self.x = prior[0]                   # State Variable
        self.P = prior[1]                   # State Covariance
        self.F = np.array([[1, 0],
                           [0, 0]])         # State Transition Function
        self.B = np.array([[self.dt, 1.]]).T # Control Function
        self.u = np.array([[None]])         # Control Input
        self.sigma_v = 0.3                  # Velocity Variance (0.15<var<0.3)
        self.Q = np.array([[0.1, 0              ],
                           [0, 0.01]])   # Process Noise Matrix
        self.R = np.array([[0.5]])          # Measurement Noise Matrix
        self.H = np.array([[1, 0]])         # Measurement Function

        self.z = np.array([[None]])         # Sensor Data
        self.x_bar = np.array([[None]])     # State Prediction

    def test_run(self, u, z):
        '''
            @brief runs verbose test of kf for one loop with given inputs

            @param[in]: sensor reading
            @param[in] vel control cmd
        '''

        self.u = np.array([[0.3]])
        self.display()
        print("---------------------------------------------------------------")
        self.predict(True)
        print("---------------------------------------------------------------")
        self.z = 0.4
        self.display()
        print("---------------------------------------------------------------")
        self.update(True)
        print("---------------------------------------------------------------")
        self.display()

    def predict(self, verbose=False):
        '''
            @brief generate predicted state from prior, cmd_vel, noise

            Assumes attributes x, P are in prior state, u is up to date with
            newest command delta. Ensure this is the case before calling function

            @param[out] update state variable x
            @param[out] update state covariance P
        '''
        self.x = np.add(np.matmul(self.F, self.x), np.matmul(self.B, self.u))
        self.P = self.P + self.Q

        if verbose:
            print("Prediction State x:")
            print(self.x)
            print("Prediction Covariance P:")
            print(self.P)

    def update(self, verbose=False):
        '''
            @brief generate estimated state from measurement, prior

            Assumes attributes x, P are in predicted state, z is up to date with
            newest sensor data. Ensure this is the case before calling function

            @param[out] update state attribute x
            @param[out] update state covariance P
        '''

        S = np.add(np.matmul(np.matmul(self.H, self.P), self.H.T), self.R)  # System Uncertainty S
        y = self.z - np.matmul(self.H, self.x)                              # residual y
        K = np.matmul(np.matmul(self.P, self.H.T), np.linalg.inv(S))        # Kalman Gain K
        self.x = self.x + np.matmul(K, y)                                   # Update State variable
        self.P = np.matmul((np.identity(2) - np.matmul(K, self.H)), self.P) # Update State Covariance

        if verbose:
            print("Sensor Covariance S:")
            print(S)
            print("Residual y:")
            print(y)
            print("Kalman Gain K:")
            print(K)
            print("Posterior State x")
            print(self.x)
            print("Posterior Covariance P")
            print(self.P)

    def step(self, u, z, verbose=False):
        '''
            @brief Runs full kf loop given sensor and cmd np arrays

            @param[in]: np array of sensor reading
            @param[in]: np array of cmd vel
            @param[out] update x_bar attr with model-predicted state
        '''

        self.u = u
        self.z = z
        self.predict(verbose)
        self.x_bar = self.x
        self.update(verbose)

    def display(self):
        '''
            @brief prints settings and state of the filter
        '''

        print("Static Filter Settings")

        print("Timestep dt:")
        print(self.dt)

        print("State Variable x:")
        print(self.x)

        print("State Covariance P")
        print(self.P)

        print("State Transition Function F:")
        print(self.F)

        print("Control Function B:")
        print(self.B)

        print("Control Input u:")
        print(self.u)

        print("Velocity Variance sigma_v:")
        print(self.sigma_v)

        print("Process Noise Matrix Q:")
        print(self.Q)

        print("Measurement Noise Matrix R:")
        print(self.R)

        print("Measurement Function H:")
        print(self.H)

        print("Current sensor value z:")
        print(self.z)

if __name__ == '__main__':

    # Runs Kalman Filter display
    x = np.array([[0., 0.]]).T      # Initial State
    P = np.array([[500, 0.],
                  [0., 500]])       # Initial Covariance

    u = np.array([[0.3]])           # Command Vel
    z = np.array([[0.2]])           # Position Sensor Val

    mdkf = KalmanFilterND((x, P))
    mkdf.test_run(u, z)
