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
4. Go to your Arduino workspace (For example, *~/Arduino/*), delete */libraries/ros_lib*
5. Run **rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries/**
6. Open "~Arduino/libraries/ros_lib/ros.h"
7. Modify to

```C
...
#elif defined(__AVR_ATmega328P__)

typedef NodeHandle_<ArduinoHardware, 4, 4, 128, 128> NodeHandle;
//typedef NodeHandle_<ArduinoHardware, 25, 25, 280, 280> NodeHandle;

#elif defined(SPARK)
...
```

8. Go back to catkin workspace and run **catkin_make jenga_end_effector_firmware_end_effector_control**
9. Plug in the Arduino board
10. Run **catkin_make jenga_end_effector_firmware_end_effector_control-upload**
11. After upload, use **rosrun rosserial_python serial_node.py /dev/ttyACM0** to communicate with the Arduino board.

## Note
* The dependent Arduino libraries are assumed to be in **_~/Arduino/libraries_**. This can be modified in *jenga_end_effector/firmware/CMakeLists.txt*
* The Arduino board used is assumed to be **MEGA2560** connecting to **/dev/ttyACM0**. This can be modified in *jenga_end_effector/firmware/CMakeLists.txt*