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
import numpy as np
from kalman_filter_1d import KalmanFilter1D
from kalman_filter_nd import KalmanFilterND
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

    def plot_data(self, x, ysets, ylabels, types, title='Robot Position over Time', xlabel='Time (s)', ylabel='Position (m)'):
        '''
            @brief plots n sets of y-data against x-data

            @param[in] x-data
            @param[in] list of y-data sets (lengths match x)
            @param[in] list of y-data labels (length matches ysets length)
            @param[in] plot type string (-=line, o=point, x=tick)
            @param[in] plot title, has a default
            @param[in] x axis label, defaults to time
            @param[in] y axis label, defaults to position
        '''
        for i in range(len(ysets)):
            plt.plot(x,ysets[i], types[i], label=ylabels[i])

        plt.legend()
        plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)

    def print_data(self,name=None):
        '''
            @brief print specified class dataset attribute (from csv)

            @param name of column, print full csv if not set
        '''
        if not name: print(self.data)
        else: print(self.data[name])

    def generate_noise(self, gaussian, dataset):
        '''
            @brief adds specified gaussian noise to dataset

            @param[in] (mean, var) to define gaussian noise profile
            @param[in] dataset to which to add noise
            @returns dataset with added noise
        '''

        return [datum + np.random.normal(gaussian[0], gaussian[1]) for datum in dataset]

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

    def generate_kalman_estimate(self, us, zs, prior):
        '''
            @brief Generates kalman filter estimate given model, sensor

            model and sensors lists should be the same length, prior should have
            2*1 vector of init pos, vel, 2*2 matrix of init covariance
            TODO(connor): automate and extend returning to a single scaleable
            entity

            @param[in] list of commands us
            @param[in] list of sensor readings zs
            @param[in] prior init (x, P)
            @return list of kf-gen pos estimates
            @return list of kf-gen pos variances
            @return list of kf-gen vel estimates
            @return list of kf-gen vel variances
            @return list of kf-gen pos model predictions
        '''
        kf = KalmanFilterND(prior)
        xs = [0] * len(zs)
        pxs = [0] * len(zs)
        vs = [0] * len(zs)
        pvs = [0] * len(zs)
        x_bars = [0] * len(zs)

        # Run kalman filter
        for i in range(0, len(zs)):
            u = np.array([[2*us[i]]])
            z = np.array([[zs[i]]])
            kf.step(u, z, False)
            xs[i] = kf.x[0]
            pxs[i] = kf.P[0,0]
            vs[i] = kf.x[1]
            pvs[i] = kf.P[1,1]
            x_bars[i] = kf.x_bar[0]

        return xs, pxs, vs, pvs, x_bars

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
        U =  self.data['Command Lin Vel'][:]
        print(U[0])
        print(U[1])
        print(U[2])
        Y =  self.data['Ground Truth X'][:]
        Y2 = self.data['Encoder X'][:]
        Y3 = self.generate_model_estimate(X, U, Y[0]) # Create model
        Y2_noisy = self.generate_noise((0, 0.5), Y2)

        ulabel = "Linear Velocity Commands"
        ylabel= "Ground Truth X"
        y2label = "Sensor Readings z"
        y2noisylabel = "Noisy Sensor Readings"
        y3label = "Model-Only Prediction"
        y4label = "KF 1-dim Model Estimate"
        y5label = "KF 1-dim Pose Estimate"
        y6label = "KF 2-dim Pose Estimate X"
        v_k2_label = "KF 2-dim Vel Estimate"
        x_k2model_label = "KF 2-dim Pose Prediction x_bar"

        # Post-process data
        X = self.post_process(X, offset=-X[0])                       # Time
        Y2 = self.post_process(Y2, offset = Y[0] - Y2[0])            # Encoder w/o flip
        Y2_noisy = self.post_process(Y2_noisy, offset = Y[0] - Y2_noisy[0])
        #Y2 = self.post_process(Y2, offset=Y[0]-Y2[0], flip_pt=Y2[0]) # Encoder
        Y3 = self.post_process(Y3, flip_pt = Y[0])                   # Model

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

        Y5 = [x for (x,y) in self.prediction]
        Y5Var = [y for (x,y) in self.prediction]

        x_init = np.array([[Y[0],0]]).T
        P_init = np.array([[0.01,0],
                           [0,0.01]])
        Y6, Y6Var, v_k2, vvar_k2, x_k2model = self.generate_kalman_estimate(U, Y2, (x_init, P_init))
        Y7, Y7Var, V7, V7Var, X7 = self.generate_kalman_estimate(U, Y2_noisy, (x_init, P_init))

        # plot new data
        self.plot_data(X, [U], ["Velocity Commands U"], "o",
                       ylabel="Velocity in X (m/s)")
        plt.show()
        self.plot_data(X,[Y2, Y6, x_k2model],
                         ["Encoder Data z", "2D KF Estimate x", "Model Estimate x_bar"],
                         "o-*", ylabel = "Position in X (m)")
        plt.show()

        # Plot velocities
        self.plot_data(X, [Y2_noisy, Y7, X7],
                      ["Noisy Encoder Data z", "2D KF Estimate x", "Model Estimate x_bar"],
                      "o-*", ylabel = "Position in X (m)")
        plt.show()

if __name__ == '__main__':

    # Init DataWithKalman with filename arg
    if len(sys.argv) == 2: node = DataWithKalman(sys.argv[1])
    else: node = DataWithKalman()

    node.generate_graph()
