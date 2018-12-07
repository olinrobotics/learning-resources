# Blog Post 1: 1D Kalman Filter Implementation
*November 30th, 2018*

In order to solidify the fundamental concepts of Kalman filters before implementing a non-linear filter for full positioning of the Neato, we planned to implement a 1-dimensional Kalman filter to track the Neato's position in a single dimension. We intended to use this process to help us design our data collection framework and class structure, making the code as modular as possible to ensure that as we created new filters, we did not have to re-write implementation code.

## Data Collection
With any comparison of different algorithmic techniques, it is useful to have a baseline for data so that you can meaningfully compare different approaches. We planned to re-build Paul's ceiling tile mounted april tag system to position the robot in the classroom. We were told that this system had a high degree of accuracy and would be good to use as a ground truth. Our setup of the system is documented in the GROUNDTRUTH_SETUP.md file.

## Class Structure
We wanted to separate the filter implementation from the class implementation of the robot. As such, we created two classes: robot and kalman_filter_1d.

### robot.py

The robot class takes care of all ROS functionality. It subscribes to all sensors and robot commands, compiles data going into and out of the robot, and stores the data in a .csv file in the `/data/` folder. the initializing arguments it takes are a filename to write to and a boolean depicting whether or not to write data. The objective of this class was to run a Kalman filter and log the position estimates in conjunction with position estimates from the sensors for comparison.

## Filter
We had planned on starting our 1d filter comparing the raw encoder estimate to the kalman filter estimate to the groundtruth estimate of position over time in the x-dimension. 
