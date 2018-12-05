#!/usr/bin/env python

'''
ROS Neato Teleoperation Node Script
@author: Connor Novak
@email: connorrnovak@gmail.com
@brief: Publishes to /arb_vel based on keyboard inputs w,a,s,d,x
@version: 0.3
'''

from __future__ import print_function, division     # Used for Python 2-3 compatibility
import tty                                          # Terminal control functions
import select                                       # Checks stream objects for data
import sys                                          # Gives information about Python interpreter
import termios                                      # POSIX (family of terminal API standards)-style tty control
import rospy                                        # ROS library for Python
from geometry_msgs.msg import Twist, Vector3        # message type to create TwistLabeled msg
from std_msgs.msg import Header                     # message type to create TwistLabeled msg

class Telemodule():

    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None
        self.keys_action = {
                        'w': (0.05, 0),
                        'a': (0, 0.15),
                        's': (-0.05, 0),
                        'd': (0, -0.15),
                        'q': (0, 0)
                        }               # keybindings for neato vel/ang cmds
        self.move_state = Twist()

        # ROS init
        rospy.init_node('teleop')
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        self.update_rate = rospy.Rate(6)

        self.print_instructions()

    def get_key(self):
        # Takes no arguments, returns pressed key

        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def key_action(self, key):
        # Takes key pressed as arg, returns 2-part tuple (x, theta)

        return self.keys_action.get(key,(0, 0))

    def key_ismember(self, key, dict):
        # Takes key pressed , dict to check, returns True if key is in dict
        if self.keys_action.get(key,None) != None:
            return True
        return False

    def update_move(self, update):
        # Takes 2-part tuple (x, theta) as arg; increments neato movement accordingly

        # If (0,0), stop robot
        if update[0] == 0 and update[1] == 0:
            self.move_state.linear.x = 0; self.move_state.angular.z = 0

        # Increment twist.linear velocity if within +/- 0.3
        if update[0] > 0 and self.move_state.linear.x < 0.3 or update[0] < 0 and self.move_state.linear.x > -0.3:
            self.move_state.linear.x += update[0]

        # Increment angular velocity if within +/- 2.5
        if update[1] > 0 and self.move_state.angular.z < 2.5 or update[1] < 0 and self.move_state.angular.z > -2.5:
            self.move_state.angular.z += update[1]

    def print_instructions(self):
        rospy.loginfo("----------| TELEOPERATION TERMINAL |----------")
        rospy.loginfo("")
        rospy.loginfo("INSTRUCTIONS: Press direction arrows to accelerate in that direction")
        rospy.loginfo("")
        rospy.loginfo("| FORWARD: W |")
        rospy.loginfo("| REVERSE: S |")
        rospy.loginfo("|   RIGHT: D |")
        rospy.loginfo("|    LEFT: A |")
        rospy.loginfo("|    STOP: Q |")

    def run(self):
        # Takes no args, updates neato movement with keypress and publishes

        while self.key != '\x03' and not rospy.is_shutdown():
            self.key = self.get_key()
            if self.key in self.keys_action:    # If key is drive command
                transform = self.key_action(self.key)
                self.update_move(transform)
                self.vel_pub.publish(self.move_state)

            else: print("ERR: Not a valid key")


if __name__ == '__main__':
    t1 = Telemodule()
    t1.run()
