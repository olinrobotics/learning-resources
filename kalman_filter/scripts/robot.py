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
from Gaussian import Gaussian           # gaussian distribution class

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
        # TODO: imu
        # TODO: laser

        # Data storage variables
        self.encoder_prev = None
        self.encoder_pos = None
        self.encoder_offset = None
        self.imu_offset = None
        self.command_curr = Twist((0,0,0),(0,0,0))
        self.truth_curr = None
        self.log_data = log_data
        print(log_data)

        # Initialize kalman filter

        # Setup writing to csv file data_file
        if self.log_data:
            rospy.loginfo("Saving data!")
            self.file_name = self.check_file(f)
            self.write_to_file = open(self.file_name, 'w')
            self.data = [['Time',
                          'Ground Truth X', 'Ground Truth Y', 'Ground Truth Theta',
                          'Encoder X', 'Encoder Y', 'Encoder Theta',
                          'IMU Accel X', 'IMU Accel Y', 'IMU Theta', 'IMU Vel Theta']]

        # Runtime checks
        self.init_subscribers()
        return

    def encoder_cb(self, msg):
        # update encoder position based on prev_encoder_val
        self.encoder_pos = msg
        return

    def command_cb(self, msg):
        # update current command
        self.command_curr = msg
        return

    def truth_cb(self, msg):
        # update ground truth position
        self.truth_curr = msg
        return

    def check_file(self, f):
        # Checks given file for existence, file extension, returns updated file

        # Get path to data folder
        data_path = os.path.join( os.path.abspath(os.path.dirname(__file__)),
                                  "../data/" )

        # Add file extension if necessary
        if not (f[-4:] == ".csv"):
            f = f + ".csv"

        # Create full filepath
        f = os.path.join( data_path, f )

        # Return file if it doesn't already exist
        if os.path.isfile( f ):
            rospy.logerr("File already exists!")
            sys.exit("File already exists!")
        else:
            rospy.loginfo("File passed checks successfully:\n" + f)
            return f

    def record_data(self):
        # Create writeable data array of position data from

        # Record time
        t = rospy.get_time()

        # Record ground truth data
        x_0 = self.truth_curr.pose.position.x
        y_0= self.truth_curr.pose.position.y
        theta_0 = euler_from_quaternion(quaternion = (
                                      self.truth_curr.pose.orientation.x,
                                      self.truth_curr.pose.orientation.y,
                                      self.truth_curr.pose.orientation.z,
                                      self.truth_curr.pose.orientation.w))[2]

        # Record encoder data
        x_1 = self.encoder_pos.pose.pose.position.x + self.encoder_offset[0]
        y_1 = self.encoder_pos.pose.pose.position.y + self.encoder_offset[1]
        theta_1 = euler_from_quaternion(quaternion = (
                                  self.encoder_pos.pose.pose.orientation.x,
                                  self.encoder_pos.pose.pose.orientation.y,
                                  self.encoder_pos.pose.pose.orientation.z,
                                  self.encoder_pos.pose.pose.orientation.w))[2]

        # Record imu data
        xacc_2 = self.imu_msg.linear_acceleration.x
        yacc_2 = self.imu_msg.linear_acceleration.y
        thetavel_2 = self.imu_msg.angular_velocity.z
        theta_2 = euler_from_quaternion(quaternion = (
                                        self.imu_msg.orientation.x,
                                        self.imu_msg.orientation.y,
                                        self.imu_msg.orientation.z,
                                        self.imu_msg.orientation.w))[2]
        ))
        self.data.append([t, x_0, y_0, theta_0,
                         x_1, y_1, theta_1,
                         xacc_2, yacc_2, theta_2, thetavel_2])

    def write_data(self, f, data):
        # Opens given csv file, writes data
        file_to_write = open(f, 'w')
        with file_to_write:
            writer = csv.writer(file_to_write)
            writer.writerows(data)

    '''
        @brief waits until data variables are initialized

        Checks encoder_pos and truth_curr attributes, prints
        status of check every 10s, returns when both are initialized
        Initializes initial offsets between poses of each datatype to
        normalize logged data
    '''
    def init_subscribers(self):
        # Halt until receiving messages from all subscribers
        i = [False, False]
        while not rospy.is_shutdown():

            # Check for encoder data
            if not self.encoder_pos == None:
                i[0] = True
                rospy.loginfo_throttle(10, "(10s) Received encoder data")
            else:
                rospy.logwarn_throttle(10, "(10s) Waiting for encoder data")

            # Check for ground truth data
            if not self.truth_curr  == None:
                i[1] = True
                rospy.loginfo_throttle(10, "(10s) Received ground truth data")
            else:
                rospy.logwarn_throttle(10, "(10s) Waiting for ground truth data")

            if sum(i) == 2:
                break

        self.encoder_offset = [self.truth_curr.pose.position.x - self.encoder_pos.pose.pose.position.x,
                               self.truth_curr.pose.position.y - self.encoder_pos.pose.pose.position.y]
        #TODO(connor): calculate IMU offset

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
