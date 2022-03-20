
import matplotlib.pyplot as plt
import numpy as np
import csvHandler
import numpy.random as random
import MEKF as MEKF 
import pyplotHandler as pH
import math
"""
This file's main purpose is testing. 

"""


#Do everything, lol

def runMEKF():
    
    #A) create fake data
    mekf = MEKF.MEKF(0.01, 2, 2)
    i = 0
    gyro_real_arr, GYRO_meas_arr = [], []
    acc_real_arr, ACC_meas_arr = [], []
    gps_real_arr1, gps_real_arr2, gps_real_arr3 = [], [], []
    orien_real1, orien_real2, orien_real3 = [], [], []
    GPS_meas_arr = []
    xt = []
    N = 1000 #nb samples
    while (i < N):
        #1. generate random gyro and acc data 
        random_gyro = 0 # 10.0 * np.sin(2*100*i*np.pi) + round(random.uniform(-1, 1), 4)
        random_acc = 10.0 * np.sin(2*50*i*np.pi) + round(random.normal(0, 1), 4)
        """
        GYRO_input = np.array( [random_gyro, 0, 0], dtype='f')
        """
        GYRO_input = np.array([random_gyro, 0, 0], dtype='f')
        ACC_input = np.array( [random_acc, 0, 9.81], dtype='f')
        
        #2. feed through process model to create gps data
        mekf.kf_predict(GYRO_input, ACC_input)
        mekf.kf_update()

        #3. GPS_measured = GPS_real + lots of noise
        GPS_meas = mekf.ra_k + random.normal(0, 2) * np.array([1, 1, 1], dtype='f')
        
        #4. IMU_measured = IMU_real + a bit of noise 
        GYRO_meas = GYRO_input + random.normal(0, 1) * np.array([1, 1, 1], dtype='f')
        ACC_meas = ACC_input + random.normal(0, 1) * np.array([1, 1, 1], dtype='f')
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
         
        #real orientation
        orien_real1.append(dcmToEuler(mekf.Cab_k)[0])
        orien_real2.append(dcmToEuler(mekf.Cab_k)[1])
        orien_real3.append(dcmToEuler(mekf.Cab_k)[2])        

    #B) feed data into MEKF
    mekf = MEKF.MEKF(0.01, 2, 2)
    position_pred1, position_pred2, position_pred3 = [], [], [] #predicted position
    orien_pred1, orien_pred2, orien_pred3 = [], [], []
    xt, pos_cov1a, pos_cov1b = [], [], []
    pos_cov2a, pos_cov2b = [], []
    pos_cov3a, pos_cov3b = [], []
    orien_cov1a, orien_cov1b = [], []
    orien_cov2a, orien_cov2b = [], []
    orien_cov3a, orien_cov3b = [], []
    i = 0
    while (i < N):
        #predict
        mekf.kf_predict(GYRO_meas_arr[i], ACC_meas_arr[i])
        
        #correct
        #mekf.kf_update()
        #mekf.kf_correct(GPS_meas_arr[i])
        #position_corr.append(mekf.ra_k[0][0])
          
        #position
        position_pred1.append(mekf.ra_k[0][0])
        position_pred2.append(mekf.ra_k[0][1])
        position_pred3.append(mekf.ra_k[0][2])
        
        #orientation
        orien_pred1.append(dcmToEuler(mekf.Cab_k)[0])
        orien_pred2.append(dcmToEuler(mekf.Cab_k)[1])
        orien_pred3.append(dcmToEuler(mekf.Cab_k)[2])  
        
        #cov position (use diag values)
        pos_cov1a.append(mekf.ra_k[0][0] + mekf.P_k[0][0])
        pos_cov1b.append(mekf.ra_k[0][0] - mekf.P_k[0][0])
        pos_cov2a.append(mekf.ra_k[0][1] + mekf.P_k[1][1])
        pos_cov2b.append(mekf.ra_k[0][1] - mekf.P_k[1][1])
        pos_cov3a.append(mekf.ra_k[0][2] + mekf.P_k[2][2])
        pos_cov3b.append(mekf.ra_k[0][2] - mekf.P_k[2][2])
        
        #cov orientation (use diag values)
        orien_cov1a.append(dcmToEuler(mekf.Cab_k)[0] + mekf.P_k[6][6])
        orien_cov1b.append(dcmToEuler(mekf.Cab_k)[0] - mekf.P_k[6][6])
        orien_cov2a.append(dcmToEuler(mekf.Cab_k)[1] + mekf.P_k[7][7])
        orien_cov2b.append(dcmToEuler(mekf.Cab_k)[1] - mekf.P_k[7][7])
        orien_cov3a.append(dcmToEuler(mekf.Cab_k)[2] + mekf.P_k[8][8])
        orien_cov3b.append(dcmToEuler(mekf.Cab_k)[2] - mekf.P_k[8][8])
        
        #update
        mekf.kf_update()
        xt.append(i)
        i += 1

    "note, I'm using the same cov on all axices. Will be fixed eventually."
    #position 
    pH.plotMEKF1axis(xt, position_pred1, gps_real_arr1, pos_cov1a, pos_cov1b, N)
    pH.plotMEKF1axis(xt, position_pred2, gps_real_arr2, pos_cov2a, pos_cov2b, N)
    pH.plotMEKF1axis(xt, position_pred3, gps_real_arr3, pos_cov3a, pos_cov3b, N)
    
    #orientation
    pH.plotMEKF1angle(xt, orien_pred1, orien_real1, orien_cov1a, orien_cov1b, N)
    pH.plotMEKF1angle(xt, orien_pred2, orien_real2, orien_cov2a, orien_cov2b, N)
    pH.plotMEKF1angle(xt, orien_pred3, orien_real3, orien_cov3a, orien_cov3b, N)

def dcmToEuler(dcm):
    return np.array([dcm[1][2], dcm[0][2], dcm[1][0]])
    
    
runMEKF()





