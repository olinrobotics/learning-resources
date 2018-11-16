# Overview
To set up a ground truth position for our robot experiments, we collaborated with Paul Ruvolo to set up his april tag localization
system that utilizes april tags on ceiling tiles to localize a Neato botvac modified with an upward-facing Raspberry Pi camera.  
<img src="https://github.com/olinrobotics/learning-resources/blob/kalman/kalman_filter/images/neato_camera.jpg" width=250/>

## Physical Setup Procedure
We printed out six 17.5 in * 17.5 in tags from the _______ april tag set and placed them on ceiling tiles around the room. We were limited to ceiling
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
This required restarting the robot such that the camera was not pointed at bright lights, making it's self-calibration too dark to see.
This program saves a .yaml file with the calibrated camera parameters. This .yaml file is saved in this repo.  
<img src="https://github.com/olinrobotics/learning-resources/blob/kalman/kalman_filter/images/camera_calib.jpg" width=250/>

### April Tag Calibration
The software setup uses two files from Paul's repositories:  
```
star_center_position_revised.py
calibrate_star_pose_revised.py
```
`star_center_position_revised.py` has a class MarkerProcessor() that manually adds a set of marker locators that represent the
ids and relative locations of the april tags in the room. Each ceiling tile is roughly 2 ft * 2 ft. We re-built the mapping of 
the ceiling tiles to match our new layout. The resulting modified files are stored in this repository. We then followed the
calibration instructions on the CompRobo17 website ([link](https://sites.google.com/site/comprobo17/projects/robot-localization/create-your-own-bag-file)).
