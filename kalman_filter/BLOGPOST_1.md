# Blog Post 1: 1D Kalman Filter Implementation
*November 30th, 2018*

In order to solidify the fundamental concepts of Kalman filters before implementing a non-linear filter for full positioning of the Neato, we planned to implement a 1-dimensional Kalman filter to track the Neato's position in a single dimension. We intended to use this process to help us design our data collection framework and class structure, making the code as modular as possible to ensure that as we created new filters, we did not have to re-write implementation code.

## Data Collection
With any comparison of different algorithmic techniques, it is useful to have a baseline for data so that you can meaningfully compare different approaches. We planned to re-build Paul's ceiling tile mounted april tag system to position the robot in the classroom. We were told that this system had a high degree of accuracy and would be good to use as a ground truth. Our setup of the system is documented in the GROUNDTRUTH_SETUP.md file.

## Class Structure
We wanted to separate the filter implementation from the class implementation of the robot. As such, we created two classes: robot and kalman_filter_1d.

### robot.py

The robot class takes care of all ROS functionality. It subscribes to all sensors and robot commands, compiles data going into and out of the robot, and stores the data in a .csv file in the `/data/` folder. the initializing arguments it takes are a filename to write to and a boolean depicting whether or not to write data. The objective of this class was to run a Kalman filter and log the position estimates in conjunction with position estimates from the sensors for comparison. However, we backed away from the realtime implementation temporarily in favor of simply logging data. Our implementation as of this blog post logs data from the encoder odometry and the april tag system, with the goal to add IMU and Command logging next.

### Filter
We had planned on starting our 1d filter comparing the raw encoder estimate to the kalman filter estimate to the groundtruth estimate of position over time in the x-dimension. The filter class itself implents the steps in the kalman filter of predicting the future state given the current state and updating the predicted state with the sensor data. These two steps are bundled into one step function that updates the kalman filter for a single timestep. The class takes an initial position guess and assumes a wide variance.


### Problems

Implementing both the april tag system and the imu system proved much more trouble than expected. The april tag system required sharing camera calibration and mapping parameter showing where the tags were located in the room. The original goal with the IMU was to connect it to the Neato and incorporate it into the Raspberry Pi. However, this task proved too time-consuming and unrelated to the project, so the shortcut of a long USB cable attached to the computer was implemented instead.
