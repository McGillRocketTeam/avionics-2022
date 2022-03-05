# -*- coding: utf-8 -*-
""" 

EKF additional features: 
            -Jacobian instead of transition matrix 

There are many sensors aboard the rocket, the current string format is: 
S,ACCx,ACCy,ACCz,GYROx,GYROy,GYROz,PRESSURE,LAT,LONG,MIN,SEC,SUBSEC,STATE,CONT,E
where S: Start, E: end, CONT: continuity (pyro). 

However, we only care about:
    -IMU: ACCx,ACCy,ACCz,GYROx,GYROy,GYROz
    -GPS: LAT,LONG
    -Barometer: PRESSURE
    -Time: MIN,SEC,SUBSEC (optional)

"""

import numpy as np
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R

class MEKF:

    """ process model
            Cab_k = Cab_k-1 . e^(T*gyro_input)
            Va_k = Va_k-1 + T . Cab_k-1 . acc_input + T . ga
            ra_k = ra_k-1 + Va_k-1 . T
            P_k = A_k-1 . P_k-1 . (A_k-1)^T + L_k-1 . Q_k-1 . (L_k-1)


            P_k = (1 - K_k . C_k ) . P_k . (1 - K_k . C_k)^T + K_k . M_k . R_k . (M_k)^T . (K_k)^T 
            K_k = P_k . (C_k)^T (C_k . P_k . (C_k)^T + M_k . R_k . (M_k)^T )^-1
            correction_term = K_k(GPS_input - ra_k)

            Cab_k = Cab_k . e^(-correction_term)
            Va_k = Va_k + correction_term
            ra_k = r_ak + correction_term
    """

    
    def __init__(self, dt, Q_init, R_init):
        #predict 
        self.Cab_k_1 = np.eye(3, dtype='f') # rotation matrix (DCM at k-1)
        self.Va_k_1 = np.eye(3, dtype='f') # initial speed
        self.ra_k_1 = np.eye(3, dtype='f') # initial position
        self.ga = np.array([0, 0, -9.81], dtype='f') # gravitational constant

            #A and B
        self.A = np.array([1], dtype='f') #state transition model (process model)
        self.T = dt #state transition model (process model) (B = T)

            #noise 
        self.Q = np.eye(3, dtype='f')*Q_init #covariance of process noise (error on prediction)
        self.R = np.eye(3, dtype='f')*R_init #covariance of obervation noise (sensor noise)
        # Q : get value when robot is static 
        # R : get value from sensor datasheet 

            #sensor intial
        self.GYRO_input = np.eye(3, dtype='f') *0.1 #initial input (GYRO)
        self.ACC_input = np.eye(3, dtype='f') *0.1 #initial input (IMU)

            #other variables
        self.Cab_k = np.eye(3, dtype='f')
        self.Va_k = np.eye(3, dtype='f')
        self.ra_k = np.eye(3, dtype='f')
        self.P_k = np.eye(3, dtype='f')
        self.P_k_1 = np.eye(3, dtype='f')
        self.L = np.eye(3, dtype='f')
        self.S1 = np.eye(3, dtype='f')
        self.S2 = np.eye(3, dtype='f')

        #correct
        self.correction_term = np.eye(3, dtype='f')
        self.K_k = np.eye(3, dtype='f')
        
            #sensor 
        self.GPS_input = np.eye(3, dtype='f')

        self.M_k = np.eye(3, dtype='f')
        self.C_k = np.eye(3, dtype='f')

        print("init done")
        
    def kf_predict(self, GYRO_input, ACC_input): 
        self.GYRO_input = GYRO_input 
        self.ACC_input = ACC_input 

        self.Cab_k = self.Cab_k_1 @ np.exp(self.T*self.GYRO_input)
        self.Va_k = self.Va_k_1 + self.T * self.Cab_k_1 @ self.ACC_input + self.T * self.ga
        self.ra_k = self.ra_k_1 + self.Va_k_1 * self.T

        "failure point, array probably [[1, 2, 3], [4, 5, 6]]"
        self.A = np.array( [[np.exp(self.T * self.GYRO_input), self.T * self.Cab_k_1 @ self.ACC_input, 0],
                             [0.0, 1.0, self.T], 
                             [0.0, 0.0, 1.0] ], dtype='f' )
        self.P_k = self.A @ self.P_k_1 @ self.A.T + self.L @ self.Q @ self.L.T
        
        

    def kf_correct(self, GPS_input):
        self.GPS_input = GPS_input 

        self.S1 = np.eye(3) - self.K_k @ self.C_k #utility
        self.S2 = self.M_k @ self.R @ self.M_k #utility 
        self.P_k = self.S1 @ self.P_k @ self.S1.T + self.K_k @ self.S2 @ self.K_k.T
        self.K_k = self.P_k @ self.C_k.T @ inv(self.C_k @ self.P_k @ self.C_k + self.S2)
        self.correction_term = self.K_k @ (self.GPS_input - self.ra_k)

        self.Cab_k = self.Cab_k @ np.exp(-self.correction_term)
        self.Va_k = self.Va_k +  self.correction_term
        self.ra_k = self.ra_k + self.correction_term
    

    




