
import matplotlib.pyplot as plt
import numpy as np
import csvHandler
import random
import MEKF as MEKF 
import pyplotHandler as pH
import math
"""
This file's main purpose is testing. 

"""

#CSV example 
"""
gyro = np.array([20, 30, 40])
acc = np.array( [1, 3, 4])
gps = np.array( [9, 0, 2])
print("gyro:" + str(acc))
csvHandler.store(acc, 'acc')
print(csvHandler.read( 0, 'acc'))
"""


#Do everything, lol

def runMEKF():
    
    #A) create fake data
    mekf = MEKF.MEKF(0.01, 1, 1)
    i = 0
    gyro_real_arr, GYRO_meas_arr = [], []
    acc_real_arr, ACC_meas_arr = [], []
    gps_real_arr1, gps_real_arr2, gps_real_arr3 = [], [], []
    GPS_meas_arr = []
    xt = []
    N = 1000 #nb samples
    while (i < N):
        #1. generate random gyro and acc data 
        random_gyro = 10 * np.sin(2*100*i*np.pi) #+ round(random.uniform(-1, 1), 4)
        random_acc = 0 # 10.0 * np.sin(2*50*i*np.pi) #+ round(random.uniform(-1, 1), 4)
        """
        GYRO_input = np.array( [random_gyro, 0, 0], dtype='f')
        """
        GYRO_input = np.array([0, 0, 0], dtype='f')
        ACC_input = np.array( [random_acc, 0, 9.81], dtype='f')
        
        #2. feed through process model to create gps data
        mekf.kf_predict(GYRO_input, ACC_input)
        mekf.kf_update()

        #3. GPS_measured = GPS_real + lots of noise
        GPS_meas = mekf.ra_k # random.uniform(-2, 2) * np.array([1, 1, 1], dtype='f')
        
        #4. IMU_measured = IMU_real + a bit of noise 
        GYRO_meas = GYRO_input #+ random.uniform(-1, 1) * np.eye(3, dtype='f')
        ACC_meas = ACC_input #+ random.uniform(-1, 1) * np.eye(3, dtype='f')
        xt.append(i)
        i += 1
        
        #5. persist IMU and GPS data (real and measured)
        #sensors
        gyro_real_arr.append(GYRO_input)
        GYRO_meas_arr.append(GYRO_meas)
        acc_real_arr.append(ACC_input)
        ACC_meas_arr.append(ACC_meas)
        GPS_meas_arr.append(GPS_meas)
        
        #real position
        gps_real_arr1.append(mekf.ra_k[0][0])
        gps_real_arr2.append(mekf.ra_k[0][1])
        gps_real_arr3.append(mekf.ra_k[0][2])
         

    #B) feed data into MEKF
    mekf = MEKF.MEKF(0.01, 1, 1)
    position_pred1, position_pred2, position_pred3 = [], [], [] #predicted position
    position_corr = [] #corrected position
    xt, cov1, cov2 = [], [], []
    i = 0
    while (i < N):
        #predict
        mekf.kf_predict(GYRO_meas_arr[i], ACC_meas_arr[i])
        position_pred1.append(mekf.ra_k[0][0])
        position_pred2.append(mekf.ra_k[0][1])
        position_pred3.append(mekf.ra_k[0][2])
        
        cov1.append(mekf.ra_k[0][0] + mekf.P_k[0][0])
        cov2.append(mekf.ra_k[0][0] - mekf.P_k[0][0])
        
        #correct
        #mekf.kf_correct(GPS_meas_arr[i])
        #position_corr.append(mekf.ra_k[0][0])
        
        #update
        mekf.kf_update()
        xt.append(i)
        i += 1

    "note, I'm using the same cov on all axices. Will be fixed eventually."
    #position 
    pH.plotMEKF1axis(xt, position_pred1, gps_real_arr1, cov1, cov2, N)
    pH.plotMEKF1axis(xt, position_pred2, gps_real_arr2, cov1, cov2, N)
    pH.plotMEKF1axis(xt, position_pred3, gps_real_arr3, cov1, cov2, N)
    
    #orientation
    
    
runMEKF()







