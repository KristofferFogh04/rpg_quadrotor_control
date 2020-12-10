#!/usr/bin/env python3

import sys, time
import numpy as np 
import pandas as pd 
import matplotlib.pyplot as plt
import math
import datetime
from sklearn.metrics import mean_squared_error



def main(argv):

    # Load data
    ground_truth_pos = np.load('ground_truth_pos_1.npy', allow_pickle=True)
    px4_pos = np.load('px4_pos_1.npy', allow_pickle=True)
    px4_vel = np.load('px4_vel_1.npy', allow_pickle=True)
    target = np.load('target_1.npy', allow_pickle=True)

    # Trim to the length of target
    diff = len(px4_pos[0]) - len(target[0])
    px4_pos = px4_pos[:,:-diff]
    px4_vel = px4_vel[:,:-diff]


    # Calculate position error
    RMSE = math.sqrt(mean_squared_error(target[0:1,:], px4_pos[0:1,:]))
    print(RMSE)


    fig1 = plt.figure()
    ax1 = plt.axes()
    ax1.plot(ground_truth_pos[0,:], ground_truth_pos[1,:])
    ax1.plot(target[0,:], target[1,:])
    ax1.plot(px4_pos[0,:], px4_pos[1,:])

    ax1.set(xlabel='x coordinate [m]', ylabel='y coordinate [m]',
            title='Husky position vs. target position for goto target')
    ax1.legend(['ground truth', 'target', 'px4_pos'])
    plt.grid()
    plt.savefig('plot1_gotoTarget.png')



if __name__ == '__main__':
    main(sys.argv)