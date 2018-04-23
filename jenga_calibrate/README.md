# UR5 Plays Jenga - jenga_calibrate

Provides camera calibration and hand-eye calibration routines.

## Author
**Chia-Hung Lin** - *clin110*

## Instructions
1. roscd jenga_calibrate/scripts
2. chmod +x set_camera_parameter.sh
3. roslaunch jenga_calibrate calibrate_camera.launch
4. Calibrate the camera using a checker board.
5. Click "Save" on the GUI.
6. Click "Commit" to exit the GUI. 
7. Find /tmp/calibrationdata.tar.gz, extract ost.yaml to jenga_calibrate/yaml