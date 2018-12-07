#!/usr/bin/env python

"""
TODO:
- Decide format of prior passed to KalmanFilter()
- FIX GAUSSIAN CLASS
"""

from collections import namedtuple
from Gaussian import Gaussian

#Gaussian = namedtuple('Gaussian', ['mean', 'variance'])
#Gaussian.__repr__ = lambda s: 'Gaussian(mean={:.3f}, variance={:.3f})'.format(s[0], s[1]) #Not necessary-- prints fancy symbols insteand of "mean" and "variance". Pretty cute though.

# Pseudocode
class KalmanFilter1D():
    @staticmethod
    def gaussian_multiply(g1, g2):
        mean = (g1.variance * g2.mean + g2.variance * g1.mean) / (g1.variance + g2.variance)
        variance = (g1.variance * g2.variance) / (g1.variance + g2.variance)
        return Gaussian(mean, variance)

    def __init__(self, prior_x=0):
        """All following measurements will be in relation to the starting point: The initial guess of where you are is extremely certain that it is where you started (a Gaussian with a """

        #setup:
        self.prior = Gaussian(prior_x,10000) # sets basically even distribution over midpoint of range

    def predict(self,prior,movement):
        # uses old position (previous posterior) and adds expected movement, with uncertainty
        x, P = prior.mean, prior.variance   # Unpack posterior Gaussian
        dx, Q = movement.mean, movement.variance    # Unpack movement Gaussian

        x = x + dx  # Update mean w/ predicted movement, no uncertainty
        P = P + Q   # Combine variances, decrease certainty                                

        posterior = Gaussian(x,P)
        return posterior # new prior of expected position from movement

    def update(self,prior,measurement):
        # gives final expected position based on Gaussians of predicted place and new measurement
        posterior = KalmanFilter1D.gaussian_multiply(measurement,prior)
        return posterior

    def step(self, z, dx, R=.001, Q=.001):
        measurement = Gaussian(z,R)
        movement = Gaussian(dx,Q)

        new_prior = self.predict(self.prior, movement)
        posterior = self.update(new_prior,measurement)
        self.prior = posterior
        return posterior.mean, posterior.variance
