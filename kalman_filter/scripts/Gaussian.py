# Defines a gaussian

from collections import namedtuple

Gaussian = namedtuple('Gaussian', ['mean', 'variance'])
Gaussian.__repr__ = lambda s: '𝒩(μ={:.3f}, 𝜎²={:.3f})'.format(s[0], s[1]) #Not necessary-- prints fancy symbols insteand of "mean" and "variance". Pretty cute though.

class Gaussian(object):
    def __init__(self,mean,variance):
        self._mean = mean
        self._variance = variance

    @property
    def mean(self):
        return self._mean

    @property
    def variance(self):
        return self._variance

    def __repr__(self):
        return lambda s: '𝒩(μ={:.3f}, 𝜎²={:.3f})'.format(s[0], s[1])

