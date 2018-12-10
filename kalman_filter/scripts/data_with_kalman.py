#!/usr/bin/env python

'''
    @file data_with_kalman.py
    @brief Displays results from kalman filter applied to data in given csv

    To use, run script with python3 and pass it a string argument representing
    the desired filename to view. Filename need not have a filetype, does need
    to be located in learning-resources/kalman_filter/share/.

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

    def plot_data(self, x, ysets, ylabels, types):
        '''
            @brief plots n sets of y-data against x-data

            @param[in] x-data
            @param[in] list of y-data sets (lengths match x)
            @param[in] list of y-data labels (length matches ysets length)
            @param[in] plot type string (-=line, o=point, x=tick)
        '''
        for i in range(len(ysets)):
            plt.plot(x,ysets[i], types[i], label=ylabels[i])

        plt.legend()
        plt.title('Robot Position over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')

    def print_data(self,name=None):
        '''
            @brief print specified class dataset attribute (from csv)

            @param name of column, print full csv if not set
        '''
        if not name: print(self.data)
        else: print(self.data[name])

    def generate_model_estimate(self, time, commands, init_pose):
        '''
            @brief Generates pose estimates from dt & vel cmds

            @param[in] array of timesteps
            @param[in] array of commanded velocities
            @param[in] initial position of robot (est)
            @return array of position values
        '''
        output = [0]*len(time)
        output[0] = init_pose
        for i in range(1,len(time)): output[i] = output[i-1] + (time[i]-time[i-1])*commands[i]
        return output

    def generate_kalman_estimate(self, model, sensor):
        '''
            @brief Generates kalman filter estimate given model, sensor

            @param[in] list of model estimates
            @param[in] list of sensor readings
            @return list of kalman filter-generated position estimates
        '''

    def post_process(self, data, offset=0.0, flip_pt=None):
        '''
            @brief post-processes given dataset based on param settings

            flips data, then offsets data

            @param[in] dataset to proceess
            @param[in] amount by which to offset dataset on y-axis (pos-dir)
            @param[in] intercept of data flip line y=y0 (None for no flip)
            return processed dataset
        '''

        if flip_pt != None: data = [j + 2*(flip_pt - j) for j in data]
        data = [i + offset for i in data]
        return data

    def generate_graph(self):
        self.read_data()

        # Unpack data into lists with headers
        X =  self.data['Time'][:]
        V =  self.data['Command Lin Vel'][:]
        Y =  self.data['Ground Truth X'][:]
        Y2 = self.data['Encoder X'][:]
<<<<<<< HEAD
        # add encoder offset
        diff = Y[0]-Y2[0]
        Y2 = [x+diff for x in Y2]
=======
        Y3 = self.generate_model_estimate(X, V, Y[0]) # Create model
>>>>>>> 2066b63cb4b3c418bc21f75aeeba04218b5c7151

        ylabel= "Ground Truth X"
        y2label = "Encoder Data"
        y3label = "Model-Only Prediction"
        y4label = 'KF Model Est'
        y5label = "KF: Enc + Model"

        # Post-process data
        X = self.post_process(X, offset=-X[0])                       # Time
        Y2 = self.post_process(Y2, offset=Y[0]-Y2[0], flip_pt=Y2[0]) # Encoder
        Y3 = self.post_process(Y3, flip_pt = Y[0])                   # Model

        # run encoder data through kalman filter in one go (not for live use)
        # prior_x = (max(Y)+min(Y))/2             #midpoint of ground truth range
        prior_x = (max(Y)+min(Y))/2               #midpoint of encoder range
        self.prediction.append((prior_x,0))
        self.kf = KalmanFilter1D(prior_x=prior_x)
        Z = Y2                                  # encoder data
<<<<<<< HEAD

        for i in range(1, len(Z)):
            z = Z[i]                          # current measurement
            dx = (Y3[i]-Y3[i-1])/(X[i]-X[i-1])  # delta model / delta t
            R = .1                            # sensor variance
=======
        Y4 = [0]*len(Z)                         # kalman filter model estimate
        for i in range(1, len(Z)):
            z = Z[i]                          # current measurement
            dx = (Y3[i]-Y3[i-1])/(X[i]-X[i-1])  # delta model / delta t
            x = self.prediction[i-1]
            Y4[i] = x[0] + dx
            R = 1                            # sensor variance
>>>>>>> 2066b63cb4b3c418bc21f75aeeba04218b5c7151
            Q = .01                            # movement variance
            new_pos = self.kf.step(z,dx,R,Q)    # get mean, variance back
            self.prediction.append(new_pos)

        encoder_means = [x for (x,y) in self.prediction]
        encoder_variances = [y for (x,y) in self.prediction]

        # plot new data
        Y5 = encoder_means
        self.plot_data(X,[Y, Y2, Y3, Y4, Y5],
                         [ylabel, y2label, y3label, y4label, y5label],
                         "---o-")
        plt.show()

if __name__ == '__main__':

    # Init DataWithKalman with filename arg
    if len(sys.argv) == 2: node = DataWithKalman(sys.argv[1])
    else: node = DataWithKalman()

    node.generate_graph()
