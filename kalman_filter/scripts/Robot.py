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
from tf.transformations import euler_from_quaternion

class Robot():

    def __init__(self, f='temp'):

        # ROS Constructs
        rospy.init_node("Robot")
        self.rate = rospy.Rate(2)

        self.encoder_sub = rospy.Subscriber("/odom", Odometry, self.encoder_cb)
        self.command_sub = rospy.Subscriber("/cmd_vel", Twist, self.command_cb)
        #self.truth_sub = rospy.Subscriber("/TODO", Twist, self.truth_cb)
        # TODO: imu
        # TODO: laser

        # Data storage variables
        self.encoder_prev = None
        self.encoder_pos = None
        self.command_curr = Twist((0,0,0),(0,0,0))

        # Initialize kalman filter

        # Setup writing to csv file data_file
        self.file_name = self.check_file(f)
        self.write_to_file = open(self.file_name, 'w')
        self.data = [['Encoder X', 'Encoder Y', 'Encoder Theta']]

        # Runtime checks
        self.init_encoder()
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
        x = self.encoder_pos.pose.pose.position.x
        y = self.encoder_pos.pose.pose.position.y
        theta = euler_from_quaternion(quaternion = (
                                      self.encoder_pos.pose.pose.orientation.x,
                                      self.encoder_pos.pose.pose.orientation.y,
                                      self.encoder_pos.pose.pose.orientation.z,
                                      self.encoder_pos.pose.pose.orientation.w))
        self.data.append([x, y, theta[2]])

    def write_data(self, f, data):
        # Opens given csv file, writes data
        file_to_write = open(f, 'w')
        with file_to_write:
            writer = csv.writer(file_to_write)
            writer.writerows(data)

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
