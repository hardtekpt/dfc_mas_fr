#!/usr/bin/env python3

import numpy as np

class EKF():

    def __init__(self, dim_x, dim_z):

        # Initial predicted state estimate
        self.x_hat = np.zeros((dim_x, 1))
        
        # Initial predicted estimate covariance matrix
        self.P = np.zeros((dim_x, dim_x))

        # State transition matrix
        self.F = np.eye(dim_x, dim_x)
        self.B = np.zeros((dim_x, dim_z))

        # State observation matrix
        self.H = np.zeros((dim_z, dim_x))

        # Covariance of the process noise
        self.Q = np.eye(dim_x, dim_x)

        # Covariance of the observation noise
        self.R = np.eye(dim_z, dim_z)


    def predict_step(self, u):
        #print(self.x_hat.shape, 'predict a', np.matmul(self.F, self.x_hat).shape, np.matmul(self.B, u).shape)
        self.x_hat = np.matmul(self.F, self.x_hat) + np.matmul(self.B, u)
        #print(self.x_hat.shape, 'predict p', self.B.shape, u.shape)
        self.P = np.matmul( np.matmul(self.F, self.P), np.transpose(self.F) ) + self.Q

    def update_step(self, z):
        
        z_pred = np.matmul(self.H, self.x_hat)
        y = z - np.transpose(z_pred)
        y = np.transpose(y)
        S = np.matmul( np.matmul(self.H, self.P), np.transpose(self.H) ) + self.R
        K = np.matmul( np.matmul(self.P, np.transpose(self.H)), np.linalg.inv(S) )

        #print(self.x_hat.shape, K.shape, y.shape)
        self.x_hat = self.x_hat + np.matmul(K, y)

        #print(self.x_hat.shape, self.x_hat)
        self.P = np.matmul( np.identity(len(self.x_hat)) - np.matmul(K, self.H), self.P )


    

        