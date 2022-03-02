
import matplotlib.pyplot as plt
import numpy as np
import csvHandler
import random
import MEKF as MEKF 
"""
This file's main purpose is testing. 

"""

""" CSV example 
data1 = np.array([20, 30, 40])
print("data1:" + str(data1))
csvHandler.store(data1)
print(csvHandler.read(0))
""" 


#Create fake IMU and GPS data 
def genData():
    mekf = MEKF.MEKF(0.1, np.eye(3), np.eye(3))
    i = 0
    while (i < 1000):
        #1. generate random gyro and acc data 
        
        
        #2. feed through process model to create gps data
        
        
        
        #3. GPS_measured = GPS_real + lots of noise
        
        #4. IMU_measured = IMU_real + a bit of noise 

        i += 1
  
 
#create instance 

#while loop 