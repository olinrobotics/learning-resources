"""This reads data from a csv and applies a kalman filter over it"""
import matplotlib.pyplot as plt
import csv
import sys
import pandas as pd
from kalman_filter_1d import KalmanFilter1D

class DataWithKalman():
    def __init__(self,f='temp'):
        # name of csv file
        self.file_name = f
        # data storage (will be pandas dataframe)
        self.data = None

        # initialize kalman filter
        self.kf = None
        self.prediction = [] # array of Gaussian tuples

    def read_data(self):
        #stores data in dataframe. Acts like a dict, can call array of values from name of variable
        df = pd.read_csv(self.file_name, index_col=None) 
        self.data = df

    def plot_data(self,x,y,y2,ylabel='1',y2label='2'):
        plt.plot(x,y,label=ylabel)
        plt.plot(x,y2,label=y2label)
        plt.legend()
        #plt.show()

    def print_data(self,name=None):
        # Prints full csv file if column not specified
        if not name:
            print(self.data)
        else:
            print(self.data[name])

    def check_keys(self):
        # Checks the names of the data taken
        for key in self.data:
            print(key)

    def run(self):
        self.read_data()

        # Set variables for plotting/filtering:
        X = self.data['Time'][:]
        Y = self.data['Ground Truth X'][:]
        Y2 = self.data['Encoder X'][:]
        ylabel= "Ground Truth X"
        y2label = "Encoder X"

        # plot raw data
        self.plot_data(X,Y,Y2,ylabel,y2label)

        # run encoder data through kalman filter in one go (not for live use)
        #prior_x = (max(Y)+min(Y))/2               #midpoint of ground truth range
        prior_x = (max(Y)+min(Y))/2               #midpoint of encoder range
        self.prediction.append((prior_x,0))
        self.kf = KalmanFilter1D(prior_x=prior_x)
        Z = Y2                                  # encoder data
        for i in range(1, len(Z)):
            z = Z[i-1]                          # current measurement
            dx = (Y[i]-Y[i-1])/(X[i]-X[i-1])    # using delta ground truth/delta t for now
            R = .001                            # sensor variance
            Q = .001                            # movement variance
            new_pos = self.kf.step(z,dx,R,Q)    # get mean, variance back
            self.prediction.append(new_pos)
        
        encoder_means = [x for (x,y) in self.prediction]
        encoder_variances = [y for (x,y) in self.prediction]

        # plot new data
        Y2 = encoder_means
        y2label = "P from Encoder X"
        self.plot_data(X,Y,Y2,ylabel,y2label)
        plt.show()


if __name__ == '__main__':
    if len(sys.argv) == 2:
        node = DataWithKalman(sys.argv[1])
    else:
        node = DataWithKalman()
    node.run()