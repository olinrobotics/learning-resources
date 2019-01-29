# Overview
To set up a ground truth position for our robot experiments, we collaborated with Paul Ruvolo to set up his april tag localization
system that utilizes april tags on ceiling tiles to localize a Neato botvac modified with an upward-facing Raspberry Pi camera.  
<img src="https://github.com/olinrobotics/learning-resources/blob/kalman/kalman_filter/images/neato_camera.jpg" width=250/>

## Physical Setup Procedure
We printed out six 17.5 in * 17.5 in tags from the 36h11 april tag set (link)[https://github.com/AprilRobotics/apriltag-imgs] and placed them on ceiling tiles around the room. We were limited to ceiling
tiles that could be fully removed, because successfully attaching the tags required taping to the backside of the tiles. All tags 
were placed with the x-axis pointed away from the doorway and the y-axis pointed towards the rear of the room. Many ceiling tiles
cannot be fully removed due to piping.  
<img src="https://github.com/olinrobotics/learning-resources/blob/kalman/kalman_filter/images/full_tags.jpg" width=250/>

Because the calibration procedure to determine the offset of the camera from the robot's 
rotational center required only one tag tag to be visible, we set up one tag in the far corner of the room as a calibration 
position.  
<img src="https://github.com/olinrobotics/learning-resources/blob/kalman/kalman_filter/images/corner_tag.jpg" width=250/>

## Software Setup Procedure

### Camera Calibration
We calibrated the Raspberry Pi camera following the normal instructions for the `camera_calibration` package ([link](http://wiki.ros.org/camera_calibration)).

From above, run this in the terminal:
`rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.108 image:=/camera/image_raw camera:=/camera`
If something goes wrong, check the names of the image topics (image:=) and the size of the calibration grid (--size).

This required restarting the robot such that the camera was not pointed at bright lights, making it's self-calibration too dark to see.

This program saves a .yaml file with the calibrated camera parameters. This .yaml file is saved by default in `comprobo18/gscam/examples/camera_parameters.yaml`.
<img src="https://github.com/olinrobotics/learning-resources/blob/kalman/kalman_filter/images/camera_calib.jpg" width=250/>

You can also run the april tag scripts like this:
`rosrun my_pf star_center_position_revised.py _pose_correction:=0.37954481972790866 _phase_offset:=0.5509349120120457`
where the pose correction and phase offset are calibrations taken with Paul. We'll include this in the april_tags.launch file next time.

### April Tag Calibration
The software setup uses two files from Paul's repositories and the ROS apriltags package:
```
star_center_position_revised.py
calibrate_star_pose_revised.py
```

1. Go to https://github.com/RIVeR-Lab/apriltags_ros and follow the installation instructions.

2. Run the `star_center_position_revised.py` 

`star_center_position_revised.py` has a class MarkerProcessor() that manually adds a set of marker locators that represent the
ids and relative locations of the april tags in the room. Each ceiling tile is roughly 2 ft * 2 ft. We re-built the mapping of 
the ceiling tiles to match our new layout. The resulting modified files are stored in this repository. We then followed the
calibration instructions on the CompRobo17 website ([link](https://sites.google.com/site/comprobo17/projects/robot-localization/create-your-own-bag-file)).
