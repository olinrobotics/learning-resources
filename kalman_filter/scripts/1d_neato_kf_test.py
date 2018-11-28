"""
test whether predict and update work appropriately with given error values
"""

import unittest
from neato_1d_kf import KalmanFilter

class KalmanFilter_Test(unittest.TestCase):
    @staticmethod
    def update(prior, measurement):
        x, P = prior        # mean and variance of prior
        z, R = measurement  # mean and variance of measurement
        
        y = z - x        # residual (diff between prior and measurement)
        K = P / (P + R)  # Kalman gain (calculated from Gaussian math -- weights new x based the difference in the trustworthiness of the prior and measurement)

        x = x + K*y      # posterior (this is where Kalman gain weights guess)
        P = (1 - K) * P  # posterior variance

        posterior = gaussian(x,P)
        return posterior

    def setUp(self):
        self.KalmanFilter = KalmanFilter()
        #helper code for gaussians:
        gaussian = namedtuple('Gaussian', ['mean', 'variance'])
        gaussian.__repr__ = lambda s: '𝒩(μ={:.3f}, 𝜎²={:.3f})'.format(s[0], s[1])

    def predict_test_no_movement(self):
        z = 0
        dx = 0
        R = 1
        Q = 1

        measurement = gaussian(z,R)
        movement = gaussian(dx,0)

        prior = gaussian(0,.001)
        new_prior = self.KalmanFilter.predict(prior,movement)

        self.assertEqual(new_prior.mean,prior.mean)
        self.assertEqual()

if __name__=='__main__':
    unittest.main()
