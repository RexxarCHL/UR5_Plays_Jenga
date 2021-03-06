# UR5 Plays Jenga - jenga_calibrate

Provides camera calibration and hand-eye calibration routines.

## Author
**Chia-Hung Lin** - *clin110*

## ROS Dependency
* usb_cam - ROS driver for usb cameras.
* camera_calibration - Used to find intrinsic parameters for the camera.
* ar_track_alvar - Used for hand-eye calibration and for tracking the location of the jenga tower.

## Linux Dependency
* v4l-utils - Driver for the Logitech C920 webcam used in this project.

## Instructions
1. **roscd jenga_calibrate/scripts**
2. **chmod +x set_camera_parameter.sh base_link_to_camera_tf_broadcaster.py**
3. **roslaunch jenga_calibrate calibrate_camera.launch**
4. Calibrate the camera using a checker board.
5. Click "Save" on the GUI.
6. Click "Commit" to exit the GUI. 
7. Find _/tmp/calibrationdata.tar.gz_, extract _ost.yaml_ to _jenga_calibrate/yaml/_
8. **roscd jenga_calibrate/scripts** if you are not there already.
9. **roslaunch jenga_calibrate/track_ar_tag.launch**
10. In another terminal, open MATLAB by running **matlab hand_eye_calibration.m**
11. Run the script and move the robot around to obtain hand-eye calibration result.
12. Confirm the result is stored in _jenga_calibrate/yaml/camera_tranformation.yaml_
13. Close MATLAB.
14. **rosrun jenga_calibrate tower_location_broadcaster**
15. Place the tracking paper in the view of the camera. Confirm the tracking result.
16. Done! Close everything.