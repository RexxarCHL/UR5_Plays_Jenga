# UR5 Plays Jenga - jenga_launch

One stop to launch everything from jenga package family except jenga player. 

## Author
**Chia-Hung Lin** - *clin110*

## Instructions
This instruction assumes you had already successfully **catkin_make** and source'd the packages.
### Camera and hand-eye calibration 
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

### Launch necessary nodes for Jenga playing
17. **roslaunch jenga_launch ur5_plays_jenga.launch**

### Start the player node
18. **rosrun jenga_player jenga_player**
19. Sit back and watch the robot play Jenga. Put your hand on the **BIG RED BUTTON** though. :wink: