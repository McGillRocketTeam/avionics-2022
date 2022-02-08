
import matplotlib.pyplot as plt

"""
This file's main purpose is testing. 

"""

def PlotKF(time, position, ylabel, col): #Plots KF
    plt.scatter(time, position, color=col, linestyle='--', linewidths=0.1, 
                marker='o', label=ylabel)
    plt.xlabel('time')
    plt.ylabel(ylabel)
    
#create instance 

#while loop 