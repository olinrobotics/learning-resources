#!/usr/bin/env python

'''
    @file data_with_kalman.py
    @brief Displays results from kalman filter applied to data in given csv

    @author Charlie Weiss
'''

import matplotlib.pyplot as plt
import csv
import sys
import pandas as pd
from kalman_filter_1d import KalmanFilter1D
from Gaussian import Gaussian
from csv_functions import *

class DataWithKalman():
    def __init__(self,f='temp'):
        '''
            @brief Init kalman filter, csv file to read

            @param csv filename to read from, defaults to temp
        '''

        self.file_name = check_file(f, 'share', 'r')  # name of csv file
        self.data = None    # data storage (will be pandas dataframe)

        # initialize kalman filter
        self.kf = None
        self.prediction = [] # array of Gaussian tuples

    def read_data(self):
        '''
            @brief stores data from file

            Stores csv data in pandas dataframe - acts like a dictionary,
            can call an array of values from name of variable (1st row of csv)

            @param[out] self.data set to pandas dataframe of csv
        '''
        df = pd.read_csv(self.file_name, index_col=None)
        self.data = df

    def plot_data(self, x, ysets, ylabels):
        '''
            @brief plots n sets of y-data against x-data

            @param[in] x-data
            @param[in] list of y-data sets (lengths match x)
            @param[in] list of y-data labels (length matches ysets length)
        '''
        for i in range(len(ysets)):
            plt.plot(x,ysets[i],label=ylabels[i])

        plt.legend()
        plt.title('Robot Position over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')

    def print_data(self,name=None):
        '''
            @brief print dataset from csv

            @param name of column, print full csv if not set
        '''
        if not name: print(self.data)
        else: print(self.data[name])

    def generate_model_estimate(self, time, commands):
        '''
            @brief Generates pose estimates from dt & vel cmds

            @param[in] array of timesteps
            @param[in] array of commanded velocities
            @return list of position values
        '''
        output = [0]*len(time)
        output[0] = self.data['Ground Truth X'][0] # Initialize position estimate from ground truth
        for i in range(1,len(time)): output[i] = output[i-1] + (time[i]-time[i-1])*commands[i]
        return output

    def generate_kalman_estimate(self, model, sensor):
        '''
            @brief Generates kalman filter estimate given model, sensor

            @param[in] list of model estimates
            @param[in] list of sensor readings
            @return list of kalman filter-generated position estimates
        '''

    def generate_graph(self):
        self.read_data()

        # Unpack data into lists with headers
        X =  self.data['Time'][:]
        V =  self.data['Command Lin Vel'][:]
        Y =  self.data['Ground Truth X'][:]
        Y2 = self.data['Encoder X'][:]
        ylabel= "Ground Truth X"
        y2label = "Encoder Data"

        # Post-process data
        X = [val - X[0] for val in X]           # zero-reference time data
        y2_offset = Y[0] - Y2[0]                # offset between encoder and groundtruth
        y2_flip = [True, False]                 # Whether or not [x,y] axes are flipped
        y_flipline_x = Y[0]                    # horizontal line y-intercept about which to flip data
        Y2 = [val + y2_offset for val in Y2]    # Fix enc-groundtruth offset
        Y2 = [val + 2*(y_flipline_x - val) for val in Y2] # Flip data about initial point horiz line

        # Generate model of robot motion
        y3label = "Model-Only Prediction"
        Y3 = self.generate_model_estimate(X, V)
        Y3 = [val + 2*(y_flipline_x - val) for val in Y3] # Flip data about initial point horiz line

        self.plot_data(X,[Y,Y2,Y3],[ylabel,y2label,y3label])

        # run encoder data through kalman filter in one go (not for live use)
        # prior_x = (max(Y)+min(Y))/2             #midpoint of ground truth range
        prior_x = (max(Y)+min(Y))/2               #midpoint of encoder range
        self.prediction.append((prior_x,0))
        self.kf = KalmanFilter1D(prior_x=prior_x)
        Z = Y2                                  # encoder data
        Y4 = [0]*len(Z)                         # kalman filter model estimate
        for i in range(1, len(Z)):
            z = Z[i]                          # current measurement
            dx = (Y3[i]-Y3[i-1])/(X[i]-X[i-1])  # delta model / delta t
            x = self.prediction[i-1]
            Y4[i] = x[0] + dx
            R = 1                            # sensor variance
            Q = .01                            # movement variance
            new_pos = self.kf.step(z,dx,R,Q)    # get mean, variance back
            self.prediction.append(new_pos)

        encoder_means = [x for (x,y) in self.prediction]
        encoder_variances = [y for (x,y) in self.prediction]

        # plot new data
        Y5 = encoder_means
        y5label = "KF: Enc + Model"
        plt.plot(X, Y4, 'o', label='KF Model Est')
        self.plot_data(X,[Y5],[y5label])
        plt.show()

if __name__ == '__main__':

    # Init DataWithKalman with filename arg
    if len(sys.argv) == 2: node = DataWithKalman(sys.argv[1])
    else: node = DataWithKalman()

    node.generate_graph()
