"""
TODO:
- Decide format of prior passed to KalmanFilter()
"""
from collections import namedtuple

# Pseudocode
class KalmanFilter():
    @staticmethod
    def gaussian_multiply(g1, g2):
        mean = (g1.var * g2.mean + g2.var * g1.mean) / (g1.var + g2.var)
        variance = (g1.var * g2.var) / (g1.var + g2.var)
        return gaussian(mean, variance)

    def __init__(self, prior):
        """All following measurements will be in relation to the starting point: The initial guess of where you are is extremely certain that it is where you started (a gaussian with a """
        #helper code for gaussians:
        gaussian = namedtuple('Gaussian', ['mean', 'variance'])
        gaussian.__repr__ = lambda s: '𝒩(μ={:.3f}, 𝜎²={:.3f})'.format(s[0], s[1])

        #setup:
        self.prior = gaussian(0,.001)

    def predict(self,prior,movement):
        # uses old position (previous posterior) and adds expected movement, with uncertainty
        x, P = prior.mean, prior.variance   # Unpack posterior gaussian
        dx, Q = movement.mean, movement.variance    # Unpack movement gaussian
        
        x = x + dx  # Update mean w/ predicted movement, no uncertainty
        P = P + Q   # Combine variances, decrease certainty

        posterior = gaussian(x,P)
        return posterior # new prior of expected position from movement

    def update(self,prior,measurement):
        # gives final expected position based on gaussians of predicted place and new measurement
        posterior = KalmanFilter.gaussian_multiply(measurement,prior)
        return posterior

    """ # alternate update() based on Kalman gain:
    def update(prior, measurement):
        x, P = prior        # mean and variance of prior
        z, R = measurement  # mean and variance of measurement
        
        y = z - x        # residual (diff between prior and measurement)
        K = P / (P + R)  # Kalman gain (calculated from Gaussian math -- weights new x based the difference in the trustworthiness of the prior and measurement)

        x = x + K*y      # posterior (this is where Kalman gain weights guess)
        P = (1 - K) * P  # posterior variance

        posterior = gaussian(x,P)
        return posterior
    """

    def step(self, z, dx, R=.001, Q=.001):
        measurement = gaussian(z,R)
        movement = gaussian(dx,Q)

        new_prior = predict(self.prior, movement)
        posterior = update(new_prior,measurement)
        self.prior = posterior
        return posterior

    





    
    