#!usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose

class ImuPoseEstimator():

    def __init__(self):

        rospy.init_node('imu_pos_estimator')
        rospy.Subscriber('/imu', Imu, self.imu_cb)
        self.pose_pub = rospy.Publisher('/imu_pose', Pose, queue_size=1)

        self.curr_imu = None
        self.curr_acc = [0.0, 0.0, 0.0]
        self.curr_estvel = [0.0, 0.0, 0.0]
        self.curr_pose = Pose()
        self.dt = rospy.get_time()
        self.prev_time = rospy.get_time()

    def imu_cb(self, msg):

        # Update dt from sensor msg
        self.dt = msg.header.stamp.to_sec() - self.prev_time
        self.prev_time = rospy.get_time()
        print(self.dt, msg.header.stamp.to_sec())
        # Update accelerations from sensor msg
        self.curr_acc = [msg.linear_acceleration.x,
                         msg.linear_acceleration.y,
                         msg.linear_acceleration.z]
        self.update_pose()
        return

    def update_pose(self):

        # Update velocity estimates based on timestep
        self.curr_estvel = [self.curr_estvel[0] + self.curr_acc[0] * self.dt,
                            self.curr_estvel[1] + self.curr_acc[1] * self.dt,
                            self.curr_estvel[2] + self.curr_acc[2] * self.dt]

        # Update position estimates based on vel estimates, timestep
        self.curr_pose.position.x = self.curr_pose.position.x + self.curr_estvel[0] * self.dt
        self.curr_pose.position.y = self.curr_pose.position.y + self.curr_estvel[1] * self.dt
        self.curr_pose.position.z = self.curr_pose.position.z + self.curr_estvel[2] * self.dt

        # Publish pose
        self.pose_pub.publish(self.curr_pose)

if __name__ == "__main__":
    estimator = ImuPoseEstimator()
    rospy.spin()
