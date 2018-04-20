# UR5 Plays Jenga - jenga_end_effector

Provides a-la-carte functionalities for the Arduino based end-effector.

## Author
**Chia-Hung Lin** - *clin110*

## ROS Dependency
* jenga_msgs - Messages needed to control the end-effector
* rosserial - Provides serial communication to Arduino board on the end-effector

## Arduino Dependency
* Adafruit_VL6180X - Arduino Library for VL6180X laser range finder
* HX711 - Arduino Library for HX711 load cell amplifier

## Instructions
1. **git clone** this repo and *jenga_msgs* repo into *${path_to_catkin_workspace}/src/*
2. Go to the root of your catkin workspace and run **catkin_make --pkg jenga_msgs**
3. **source devel/setup.bash**
3. Go to your Arduino workspace (For example, *~/Arduino/*), delete */libraries/ros_lib*
4. Run **rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries/**
3. Go back to catkin workspace and run **catkin_make jenga_end_effector_firmware_end_effector_control**
4. Plug in the Arduino board
5. Run **catkin_make jenga_end_effector_firmware_end_effector_control-upload**

## Note
* The dependent Arduino libraries are assumed to be in **_~/Arduino/libraries_**. This can be modified in *jenga_end_effector/firmware/CMakeLists.txt*
* The Arduino board used is assumed to be **MEGA2560** connecting to **/dev/ttyACM0**. This can be modified in *jenga_end_effector/firmware/CMakeLists.txt*