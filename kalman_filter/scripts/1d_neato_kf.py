"""
TODO:
- Decide format of prior passed to KalmanFilter()
"""
from collections import namedtuple

#helper code:
gaussian = namedtuple('Gaussian', ['mean', 'variance'])
gaussian.__repr__ = lambda s: '𝒩(μ={:.3f}, 𝜎²={:.3f})'.format(s[0], s[1])

# Pseudocode
class KalmanFilter():
    @staticmethod
    def gaussian_multiply(g1, g2):
        mean = (g1.var * g2.mean + g2.var * g1.mean) / (g1.var + g2.var)
        variance = (g1.var * g2.var) / (g1.var + g2.var)
        return gaussian(mean, variance)

    def __init__(self, prior):
        #prior should be gaussian(0,(length_track*1.)**2), which gives basically even distribution of all places within the track, which is centered around 0. This is because with Gaussians ~99.7% of values fall within  ±3𝜎  of the mean.
        self.prior = gaussian(0,1) """INCOMPLETE STATEMENT"""

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

    def step(self, x, dx, R=R_0, Q=Q_0):
        measurement = gaussian(z,R)
        movement = gaussian(dx,Q)

        new_prior = predict(self.prior, movement)
        posterior = update(new_prior,measurement)
        self.prior = posterior
        return posterior

    





    
    