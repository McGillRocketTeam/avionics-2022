# -*- coding: utf-8 -*-
"""
NOTE : THIS IS AN OLD KF CODE. I HAVE YET TO MAKE IT INTO AN EKF. 
"""

import numpy as np
from numpy.linalg import inv

class KF:

    
    def __init__(self, r1_init, dt, Q_init, R_init):
        #predict 
        self.A = np.array([1]) #state transition model (process model)
        self.B = np.array([dt]) #state transition model (process model)
        
        self.x = np.array([r1_init]) #state estimate 
        self.u = np.array([0])  #input (like IMU)
        #x_1 = A * x(t) + B * u(t)
        #r_1 = 1 * r   +  dt * v_1
        
        #correct
        self.P = np.eye(1) #covariance matrix
        self.Q = np.eye(1)*Q_init #covariance of process noise (error on prediction)
        self.R = np.eye(1)*R_init #covariance of obervation noise (sensor noise)
        # Q : get value when robot is static 
        # R : get value from sensor datasheet 
        
        self.H = np.eye(1) #observation model
        # z = measurement 
        #r_1 = r_1_measured (like a GPS)
        
        print("init done")
        
    def kf_predict(self, v_1): #works! 
        self.u = v_1
        
        #x_1 = A * x(t) + B * u(t)
        self.x = self.A.dot(self.x) + self.B.dot(self.u)
        
        # P = A*P*A.t + Q
        self.P = np.array(self.A.dot(self.P).dot(self.A.T) + self.Q)
        

    def kf_correct(self, z):
        self.z = np.array([z]) #measurement 
        
        # y = z - H * x
        self.y = self.z - self.H.dot(self.x) #innovation
        
        # S = H*P*H.t + R 
        self.s = np.array(self.H.dot(self.P).dot(self.H.T) + self.R) #innovation covariance 
        
        # K = P * H.t * S^-1
        S_inv = inv(self.s)
        self.K = self.P.dot(self.H.T).dot(S_inv) #Kalman gain 
        
        # x = x + K*y
        self.x = self.x + self.K.dot(self.y)
        
        # P = (I -K*H) * P 
        self.P = (np.eye(1) - self.K.dot(self.H)).dot(self.P)
        
    

    




