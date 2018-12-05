# Overview

Making a bag file and visualizing it in rviz can be an inherently frustrating experience. Here's how we made it work.

Here are the specific topics we wanted to record (other default rosbag topics were thrown in as well):
- /STAR_pose (ground truth)
- /STAR_pose_continuous (ground truth)
- /odom (encoders)
- /imu (imu, naturally)

## Physical Setup
Ideally, we could plug in the IMU directly into the Neato's Raspi so that it gets thrown in with the rest of the sensors. Unfortunately, the Raspi believes the IMU is the actual robot, so that doesn't work.

Instead, we taped the IMU to the Neato, and ran a long USB cable from it to a laptop for recording. 

[picture]

## Recording
Before recording the file, you'll need some setup:

1. Make sure you're connected to the Neato.

2. Make sure you're in the bags file directory, or wherever you'd like to save the bag file.

3. Run the launch files for the april tags and imu. This makes sure that those pieces of data are actually being published. If you've set everything up according to the instructions in IMU_SETUP.md and GROUNDTRUTH_SETUP.md, it should be as simple as this (make sure to run them in separate terminals! They need to stay running while you record):

`roslaunch kalman_filter apriltags.launch 
roslaunch razor_imu_9dof razor-pub.launch 
`
You can double check that data is being published by running these (again, in separate terminals):

`rostopic echo STAR_pose_continuous
rostopic echo imu
`
You should see position and orientation values for the STAR_pose, and angular and linear velocities and covariances for the imu updating very quickly.

4. Finally, you get to record the bag file! Run this in a separate terminal: 

`rosbag record -a -x"/camera/(.*)" -o name_of_bag_file`

Make sure to include the -x"/camera/(.*)". This prevents rosbag from trying to record a bunch of broken camera data. Without this, you'll get a bunch of red errors complaining about compressed camera files.

And that should be it!