#!/usr/bin/env python

'''
    @file robot.py
    @brief Neato class for collecting position estimation data using
    various sensors and filters.

    Listens to odometry, STAR pose, imu, and uses kalman filter to
    approximate position. Logs all datapoints to excel file.
    TODO(connor): Add parameters to choose which sensors to log
    TODO(connor): Add rostime to logging

    @author Connor Novak
    @date 2018-11-28
'''

import os                               # File interaction library
import sys                              # Used for program exit
import csv                              # CSV file interaction library
import rospy                            # Python ROS library
from nav_msgs.msg import Odometry       # Encoder message type
from geometry_msgs.msg import Twist     # Robot command message type
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu
from Gaussian import Gaussian           # gaussian distribution class
from csv_functions import *             # csv file usage functions

'''
    @brief main robot class for listening to topics, running kf, and logging
    results.
'''

class Robot():

    '''
        @brief initializes subscribers, checks filename validity, waits for data
    '''
    def __init__(self, f='temp', log_data=False):

        # ROS Constructs
        rospy.init_node("robot")
        self.rate = rospy.Rate(2)

        self.encoder_sub = rospy.Subscriber("/odom", Odometry, self.encoder_cb)
        self.command_sub = rospy.Subscriber("/cmd_vel", Twist, self.command_cb)
        self.truth_sub = rospy.Subscriber("/STAR_pose_continuous", PoseStamped, self.truth_cb)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_cb)
        # TODO: laser

        self.log_data = log_data

        # Data storage variables
        self.encoder_msg = None
        self.imu_msg = None
        self.star_msg = None
        self.command_msg = None

        # Setup writing to csv file data_file
        if self.log_data:
            self.file_name = self.check_file(f, "data", 'w')
            rospy.loginfo("Logging data to kalman_filter/data/%s", self.file_name)
            self.write_to_file = open(self.file_name, 'w')
            self.data = [['Time',
                          'Ground Truth X', 'Ground Truth Y', 'Ground Truth Theta',
                          'Encoder X', 'Encoder Y', 'Encoder Theta',
                          'IMU Accel X', 'IMU Accel Y', 'IMU Theta', 'IMU Vel Theta',
                          'Command Lin Vel', 'Command Ang Vel']]

        # Runtime checks
        self.init_subscribers()
        return

    def encoder_cb(self, msg):
        # update encoder position based on prev_encoder_val
        self.encoder_msg = msg
        return

    def command_cb(self, msg):
        # update current command
        self.command_msg = msg
        return

    def truth_cb(self, msg):
        # update ground truth position
        self.star_msg = msg
        return

    def imu_cb(self, msg):
        # update imu message
        self.imu_msg = msg
        return

    def record_data(self):
        # Create writeable data array of position data from

        # Record time
        t = rospy.get_time()

        # Record ground truth data
        x_0 = self.star_msg.pose.position.x
        y_0= self.star_msg.pose.position.y
        theta_0 = euler_from_quaternion(quaternion = (
                                      self.star_msg.pose.orientation.x,
                                      self.star_msg.pose.orientation.y,
                                      self.star_msg.pose.orientation.z,
                                      self.star_msg.pose.orientation.w))[2]

        # Record encoder data
        x_1 = self.encoder_msg.pose.pose.position.x
        y_1 = self.encoder_msg.pose.pose.position.y
        theta_1 = euler_from_quaternion(quaternion = (
                                  self.encoder_msg.pose.pose.orientation.x,
                                  self.encoder_msg.pose.pose.orientation.y,
                                  self.encoder_msg.pose.pose.orientation.z,
                                  self.encoder_msg.pose.pose.orientation.w))[2]

        # Record imu data
        xacc_2 = self.imu_msg.linear_acceleration.x
        yacc_2 = self.imu_msg.linear_acceleration.y
        thetavel_2 = self.imu_msg.angular_velocity.z
        theta_2 = euler_from_quaternion(quaternion = (
                                        self.imu_msg.orientation.x,
                                        self.imu_msg.orientation.y,
                                        self.imu_msg.orientation.z,
                                        self.imu_msg.orientation.w))[2]

        linvel_3 = self.command_msg.linear.x
        angvel_3 = self.command_msg.angular.z

        self.data.append([t, x_0, y_0, theta_0,
                         x_1, y_1, theta_1,
                         xacc_2, yacc_2, theta_2, thetavel_2,
                         linvel_3, angvel_3])

    def write_data(self, f, data):
        # Opens given csv file, writes data
        file_to_write = open(f, 'w')
        with file_to_write:
            writer = csv.writer(file_to_write)
            writer.writerows(data)

    '''
        @brief waits until data variables are initialized

        Checks encoder_msg and star_msg attributes, prints
        status of check every 10s, returns when both are initialized
    '''
    def init_subscribers(self):
        # Halt until receiving messages from all subscribers
        i = [0, 0, 0, 0]
        while not rospy.is_shutdown():

            # Check for encoder data
            if not self.encoder_msg == None:
                i[0] = True
                rospy.loginfo_throttle(10, "(10s) Received encoder data")
            else:
                rospy.logwarn_throttle(10, "(10s) Waiting for encoder data")

            # Check for ground truth data
            if not self.star_msg  == None:
                i[1] = True
                rospy.loginfo_throttle(10, "(10s) Received ground truth data")
            else:
                rospy.logwarn_throttle(10, "(10s) Waiting for ground truth data")

            # Check for imu data
            if not self.imu_msg == None:
                i[2] = True
                rospy.loginfo_throttle(10, "(10s) Received imu data")
            else:
                rospy.logwarn_throttle(10, "(10s) Waiting for imu data")

            # Check for imu data
            if not self.command_msg == None:
                i[3] = True
                rospy.loginfo_throttle(10, "(10s) Received command data")
            else:
                rospy.logwarn_throttle(10, "(10s) Waiting for command data")

            if sum(i) == 4:
                rospy.loginfo("All sensors initialized")
                break

    def run(self):
        while not rospy.is_shutdown():
            # Run kalman filter loop; save new estimate
            # Record estimates:
            if self.log_data: self.record_data()
            self.rate.sleep()

        if self.log_data: self.write_data(self.file_name, self.data)

if __name__ == "__main__":
    if len(sys.argv) == 3:
        Neato = Robot(sys.argv[1], int(sys.argv[2]))
        Neato.run()
    elif len(sys.argv) == 2:
        Neato = Robot(sys.argv[1])
        Neato.run()
    else:
        Neato = Robot()
        Neato.run()
