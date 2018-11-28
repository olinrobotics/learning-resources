"""
test whether predict and update work appropriately with given error values
"""

import unittest
from 1d_neato_kf import KalmanFilter

class KalmanFilter_Test(object):
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

    def predict_test(self):
        z = 0
        dx = 1
        R = 1
        Q = 1
        prior = self.KalmanFilter()
        self.assertEqual()

if __name__=='__main__':
    unnittest.main()
