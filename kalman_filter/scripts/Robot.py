'''
    @file Robot.py
    @author Connor Novak
    @date 2018-11-28

    Class to collect data from Neato regarding position estimations using
    various sensors and filters.
'''

import os                               # File interaction library
import sys                              # Used for program exit
import csv                              # CSV file interaction library
import rospy                            # Python ROS library
from nav_msgs.msg import Odometry       # Encoder message type
from geometry_msgs.msg import Twist     # Robot command message type
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class Robot():

    def __init__(self, f='temp'):

        # ROS Constructs
        rospy.init_node("Robot")
        self.rate = rospy.Rate(2)

        self.encoder_sub = rospy.Subscriber("/odom", Odometry, self.encoder_cb)
        self.command_sub = rospy.Subscriber("/cmd_vel", Twist, self.command_cb)
        self.truth_sub = rospy.Subscriber("/STAR_pose_continuous", PoseStamped, self.truth_cb)
        # TODO: imu
        # TODO: laser

        # Data storage variables
        self.encoder_prev = None
        self.encoder_pos = None
        self.command_curr = Twist((0,0,0),(0,0,0))
        self.truth_curr = None

        # Initialize kalman filter

        # Setup writing to csv file data_file
        self.file_name = self.check_file(f)
        self.write_to_file = open(self.file_name, 'w')
        self.data = [['Encoder X', 'Encoder Y', 'Encoder Theta']]

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

        # Record encoder data
        x_0 = self.encoder_pos.pose.pose.position.x
        y_0 = self.encoder_pos.pose.pose.position.y
        theta_0 = euler_from_quaternion(quaternion = (
                                      self.encoder_pos.pose.pose.orientation.x,
                                      self.encoder_pos.pose.pose.orientation.y,
                                      self.encoder_pos.pose.pose.orientation.z,
                                      self.encoder_pos.pose.pose.orientation.w))


        # Record ground truth data
        x_1 = self.truth_curr.pose.position.x
        y_1= self.truth_curr.pose.position.y
        theta_1 = euler_from_quaternion(quaternion = (
                                      self.truth_curr.pose.orientation.x,
                                      self.truth_curr.pose.orientation.y,
                                      self.truth_curr.pose.orientation.z,
                                      self.truth_curr.pose.orientation.w))
        self.data.append([x_0, y_0, theta_0, x_1, y_1, theta_1])

    def write_data(self, f, data):
        # Opens given csv file, writes data
        file_to_write = open(f, 'w')
        with file_to_write:
            writer = csv.writer(file_to_write)
            writer.writerows(data)

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

    def init_encoder(self):
        # Halt until receiving odometry messages from encoders
        while self.encoder_pos == None and not rospy.is_shutdown():
            rospy.logwarn_throttle(5, "(5s) Waiting for encoder data")
            self.rate.sleep();

        # If loop exit due to data received, msg so and update prev
        if not rospy.is_shutdown():
            rospy.loginfo("Received encoder data")
            self.encoder_prev = self.encoder_pos
        return

    def run(self):
        while not rospy.is_shutdown():
            # Run kalman filter loop; save new estimate
            # Record estimates:
            self.record_data()
            self.rate.sleep()

        self.write_data(self.file_name, self.data)

if __name__ == "__main__":
    Neato = Robot()
    Neato.run()
