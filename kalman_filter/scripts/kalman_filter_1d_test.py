"""
test whether predict and update work appropriately with random integers for mean and variance among initial prior, measurement, and movement
"""

import unittest
from collections import namedtuple
from kalman_filter_1d import KalmanFilter
import random
#from Gaussian import Gaussian


Gaussian = namedtuple('Gaussian', ['mean', 'variance'])
Gaussian.__repr__ = lambda s: '𝒩(μ={:.3f}, 𝜎²={:.3f})'.format(s[0], s[1]) #Not necessary-- prints fancy symbols insteand of "mean" and "variance". Pretty cute though.

class KalmanFilter_Test(unittest.TestCase):
    @staticmethod
    def update(prior, measurement):
        """Alternate method for gaussian multiplication to compare to one used in KalmanFilter"""
        x, P = prior.mean, prior.variance        # mean and variance of prior
        z, R = measurement.mean, measurement.variance  # mean and variance of measurement
        
        y = z - x        # residual (diff between prior and measurement)
        K = P / (P + R)  # Kalman gain (calculated from Gaussian math -- weights new x based the difference in the trustworthiness of the prior and measurement)

        x = x + K*y      # posterior (this is where Kalman gain weights guess)
        P = (1 - K) * P  # posterior variance

        posterior = Gaussian(x,P)
        return posterior

    def setUp(self):
        self.KalmanFilter = KalmanFilter()

    def test_predict(self):
        dx = random.randint(0,1000) # movement step
        Q = random.randint(0,1000) # movement variance

        movement = Gaussian(dx,Q)

        prior = Gaussian(random.randint(0,1000),random.randint(0,1000))
        new_prior = self.KalmanFilter.predict(prior,movement)

        self.assertEqual(prior.mean+movement.mean,new_prior.mean) # when means are equal, should be the same
        self.assertEqual(prior.variance+movement.variance,new_prior.variance) # variances should add

    def test_update(self):
        z = random.randint(0,1000) #measurement
        R = random.randint(0,1000) #faith in measurement

        measurement = Gaussian(z,R)

        prior = Gaussian(random.randint(0,1000),random.randint(0,1000))
        new_prior = self.KalmanFilter.update(prior,measurement)

        check_prior = KalmanFilter_Test.update(prior,measurement)

        self.assertAlmostEqual(new_prior,check_prior)

if __name__ == '__main__':
    unittest.main()
