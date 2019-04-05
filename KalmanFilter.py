from copy import deepcopy
from math import log, exp, sqrt
import sys
import warnings
import numpy as np
from numpy import dot, zeros, eye, isscalar, shape
import numpy.linalg as linalg


class KalmanFilter(object):   
    
    def reshape_z(self, z, dim_z, ndim):
        """ ensure z is a (dim_z, 1) shaped vector"""

        z = np.atleast_2d(z)
        if z.shape[1] == dim_z:
            z = z.T

        if z.shape != (dim_z, 1):
            raise ValueError('z must be convertible to shape ({}, 1)'.format(dim_z))

        if ndim == 1:
            z = z[:, 0]

        if ndim == 0:
            z = z[0, 0]

        return z

    def predict(self, u=None, B=None, F=None, Q=None):

        if B is None:
            B = self.B
        if F is None:
            F = self.F
        if Q is None:
            Q = self.Q
        elif isscalar(Q):
            Q = eye(self.xDimension) * Q

        # x = Fx + Bu
        if B is not None and u is not None:
            self.x = dot(F, self.x) + dot(B, u)
        else:
            self.x = dot(F, self.x)

        # P = FPF' + Q
        self.P = self._alpha_sq * dot(dot(F, self.P), F.T) + Q

        # save prior
        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()


    def update(self, z, R=None, H=None):

        # set to None to force recompute
        self._log_likelihood = None
        self._likelihood = None
        self._mahalanobis = None

        if z is None:
            self.z = np.array([[None]*self.zDimension]).T
            self.x_post = self.x.copy()
            self.P_post = self.P.copy()
            self.y = zeros((self.zDimension, 1))
            return

        z = self.reshape_z(z, self.zDimension, self.x.ndim)

        if R is None:
            R = self.R
        elif isscalar(R):
            R = eye(self.zDimension) * R

        if H is None:
            H = self.H

        # y = z - Hx
        # error (residual) between measurement and prediction
        self.y = z - dot(H, self.x)

        # common subexpression for speed
        PHT = dot(self.P, H.T)

        # S = HPH' + R
        # project system uncertainty into measurement space
        self.S = dot(H, PHT) + R
        self.SI = self.inv(self.S)
        # K = PH'inv(S)
        # map system uncertainty into kalman gain
        self.K = dot(PHT, self.SI)

        # x = x + Ky
        # predict new x with residual scaled by the kalman gain
        self.x = self.x + dot(self.K, self.y)

        # P = (I-KH)P(I-KH)' + KRK'
        # This is more numerically stable
        # and works for non-optimal K vs the equation
        # P = (I-KH)P usually seen in the literature.

        I_KH = self._I - dot(self.K, H)
        self.P = dot(dot(I_KH, self.P), I_KH.T) + dot(dot(self.K, R), self.K.T)

        # save measurement and posterior state
        self.z = deepcopy(z)
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

    

    
    def __init__(self, xDimension, zDimension):
        if xDimension < 1:
            raise ValueError('xDimension must be 1 or greater')
        if zDimension < 1:
            raise ValueError('zDimension must be 1 or greater')

        self.xDimension = xDimension
        self.zDimension = zDimension

        self.x = zeros((xDimension, 1))        # state
        self.P = eye(xDimension)               # uncertainty covariance
        self.Q = eye(xDimension)               # process uncertainty
        self.B = None                     # control transition matrix
        self.F = eye(xDimension)               # state transition matrix
        self.H = zeros((zDimension, xDimension))    # Measurement function
        self.R = eye(zDimension)               # state uncertainty
        self._alpha_sq = 1.               # fading memory control
        self.M = np.zeros((zDimension, zDimension)) # process-measurement cross correlation
        self.z = np.array([[None]*self.zDimension]).T

        # gain and residual are computed during the innovation step. We
        # save them so that in case you want to inspect them for various
        # purposes
        self.K = np.zeros((xDimension, zDimension)) # kalman gain
        self.y = zeros((zDimension, 1))
        self.S = np.zeros((zDimension, zDimension)) # system uncertainty
        self.SI = np.zeros((zDimension, zDimension)) # inverse system uncertainty

        # identity matrix. Do not alter this.
        self._I = np.eye(xDimension)

        # these will always be a copy of x,P after predict() is called
        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()

        # these will always be a copy of x,P after update() is called
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

        # Only computed only if requested via property
        self._log_likelihood = log(sys.float_info.min)
        self._likelihood = sys.float_info.min
        self._mahalanobis = None

        self.inv = np.linalg.inv