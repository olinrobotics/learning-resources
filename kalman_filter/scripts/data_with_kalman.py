"""This reads data from a csv and applies a kalman filter over it"""
import matplotlib.pyplot as plt
import csv
import sys
import pandas as pd
from kalman_filter_1d import KalmanFilter1D
from Gaussian import Gaussian

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
        # Prints full csv file if column not specified
        if not name:
            print(self.data)
        else:
            print(self.data[name])

    def check_keys(self):
        # Checks the names of the data taken
        for key in self.data:
            print(key)

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

    def run(self):
        self.read_data()

        # Unpack data into lists with headers
        X =  self.data['Time'][:]
        V =  self.data['Command Lin Vel'][:]
        Y =  self.data['Ground Truth X'][:]
        Y2 = self.data['Encoder X'][:]
        ylabel= "Ground Truth X"
        y2label = "Encoder X"

        # Post-process data
        X = [val - X[0] for val in X]           # zero-reference time data
        y2_offset = Y[0] - Y2[0]                # offset between encoder and groundtruth
        y2_flip = [True, False]                 # Whether or not [x,y] axes are flipped
        y_flipline_x = Y[0]                    # horizontal line y-intercept about which to flip data
        Y2 = [val + y2_offset for val in Y2]    # Fix enc-groundtruth offset
        Y2 = [val + 2*(y_flipline_x - val) for val in Y2] # Flip data about initial point horiz line

        # Generate model of robot motion
        y3label = "Model X"
        Y3 = self.generate_model_estimate(X, V)
        Y3 = [val + 2*(y_flipline_x - val) for val in Y3] # Flip data about initial point horiz line

        self.plot_data(X,[Y,Y2,Y3],[ylabel,y2label,y3label])

        # run encoder data through kalman filter in one go (not for live use)
        # prior_x = (max(Y)+min(Y))/2             #midpoint of ground truth range
        prior_x = (max(Y)+min(Y))/2               #midpoint of encoder range
        self.prediction.append((prior_x,0))
        self.kf = KalmanFilter1D(prior_x=prior_x)
        Z = Y2                                  # encoder data
        for i in range(1, len(Z)):
            z = Z[i-1]                          # current measurement
            dx = (Y3[i]-Y3[i-1])/(X[i]-X[i-1])  # delta model / delta t
            R = 1                            # sensor variance
            Q = .001                            # movement variance
            new_pos = self.kf.step(z,dx,R,Q)    # get mean, variance back
            self.prediction.append(new_pos)

        encoder_means = [x for (x,y) in self.prediction]
        encoder_variances = [y for (x,y) in self.prediction]

        # plot new data
        Y2 = encoder_means
        y2label = "KF: Enc + Model"
        self.plot_data(X,[Y2],[y2label])
        plt.show()

if __name__ == '__main__':
    if len(sys.argv) == 2:
        node = DataWithKalman(sys.argv[1])
    else:
        node = DataWithKalman()
    node.run()
