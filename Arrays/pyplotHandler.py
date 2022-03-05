

from matplotlib import pyplot as plt


def PlotKF(time, position, ylabel, color): #Plots KF
    plt.scatter(time, position, color=color, linestyle='--', linewidths=0.1, 
                marker='o', label=ylabel)
    plt.xlabel('time')
    plt.ylabel(ylabel)


