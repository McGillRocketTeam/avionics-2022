
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
    gps_real_arr, GPS_meas_arr = [], []
    xt = []
    while (i < 100):
        #1. generate random gyro and acc data 
        random_gyro = 10 * np.sin(2*100*i*np.pi) #+ round(random.uniform(-1, 1), 4)
        random_acc = 10.0 * np.sin(2*50*i*np.pi) #+ round(random.uniform(-1, 1), 4)
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
        gyro_real_arr.append(GYRO_input)
        GYRO_meas_arr.append(GYRO_meas)
        acc_real_arr.append(ACC_input)
        ACC_meas_arr.append(ACC_meas)
        gps_real_arr.append(mekf.ra_k)
        GPS_meas_arr.append(GPS_meas)
    #print("gyro_real " + str(gyro_real_arr[4]))
    #print("gyro_meas " + str(GYRO_meas_arr[4]))
    #print("gps_real" + str(gps_real_arr[4]))
        
    """
        #5. persist IMU and GPS data (real and measured)
        gyro_real_arr.append(GYRO_input[0])
        GYRO_meas_arr.append(GYRO_meas[0])
        acc_real_arr.append(ACC_input[0])
        ACC_meas_arr.append(ACC_meas[0])
        gps_real_arr.append(mekf.ra_k[0])
        GPS_meas_arr.append(GPS_meas)
    #print("gyro_real " + str(gyro_real_arr[4]))
    #print("gyro_meas " + str(GYRO_meas_arr[4]))
    #print("gps_real" + str(gps_real_arr[4]))
    pH.PlotKF(xt[0:90], ACC_meas_arr[0:90], "ACC input" , "blue")
    plt.show()
    pH.PlotKF(xt[0:90], GYRO_meas_arr[0:90], "GYRO input", "red")
    """
    """
    #file persistence
    csvHandler.store(gyro_real_arr, 'gyro_real')
    csvHandler.store(GYRO_meas_arr, 'gyro_meas')
    csvHandler.store(acc_real_arr, 'acc_real')
    csvHandler.store(ACC_meas_arr, 'acc_meas')
    csvHandler.store(gps_real_arr, 'gps_real')
    csvHandler.store(GPS_meas_arr, 'gps_meas')
    print("done creating fake data")
    """
    #B) feed data into MEKF
    position_pred = [] #predicted position
    position_corr = [] #corrected position
    xt = []
    i = 0
    while (i < 100):
        mekf.kf_predict(GYRO_meas_arr[i], ACC_meas_arr[i])
        position_pred.append(mekf.ra_k[0][0])
        #mekf.kf_correct(GPS_meas_arr[i])
        #position_corr.append(mekf.ra_k[0][0])
        mekf.kf_update()
        xt.append(i)
        i += 1
    
    #C) plot the results along a single axis
    #print("position prediction " +  str(position_pred[0:3] ))
    #print("position corrected " + str(position_corr[0:9]))
    #print(xt[0:9])
    #print(position_pred[0:9][0])
    pH.PlotKF(xt[0:1], position_pred[0:1], "predicted position x-axis", "blue")
    pH.PlotKF(xt[0:1], gps_real_arr[0:1], "real position", "red")
    
runMEKF()

