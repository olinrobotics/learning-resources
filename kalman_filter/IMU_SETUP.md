# Overview
The IMU gave us a lot of trouble in this lab. We initially tried to integrate the IMU into the Raspi onboard the Neato. However, this quickly proved to be a lot of work that was not conducive to the learning of the project, so we pivoted to the less permanent solution of taping the IMU to the robot and connecting it to the computer via a long USB extension cable.

# Attempt 1: Raspi Integration
### Upload Default Firmware

1. Go to ROS razor_imu_9dof wiki page ([link](http://wiki.ros.org/razor_imu_9dof)) and clone the source git repository.
2. Follow the instructions in the README ([link](https://github.com/KristofRobot/razor_imu_9dof)). The HW__VERSION_CODE is 10736, according to the Sparkfun page for the Razor IMU ([link](https://www.sparkfun.com/products/retired/10736)).
3. In the firmware in file `Razor_AHRS`, set your startup mode to `OUTPUT__MODE_ANGLES_AG_SENSORS` as shown below:
```
// Select your startup output mode and format here!
int output_mode = OUTPUT__MODE_ANGLES_AG_SENSORS;
int output_format = OUTPUT__FORMAT_TEXT;
```
Upload the code to the IMU and open the Serial monitor. You should see full RPYAG data being printed out.

### Setup with Neato
In imu_redirect, remove the timeout  
ssh into pi@192.168.17.209  No Name Jane 3  
~pi/CompRoboPrep/PiSetupFiles/imu_redirect.py, comment out imu redirect  
port 7778  
Robot 
https://github.com/xiaozhengxu/razor_imu_9dof
edit etc/rc.local

# Attempt 2: USB Cable

1. Follow step 1 for Attempt 1
2. Follow step 4.3 in ROS razor_imu_9d0f wiki page (linked from step 1 in Attempt 1) to setup your razor config yaml file.
3. Run launch file
