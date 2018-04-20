# UR5 Plays Jenga
Independent project for ME530.707 Spring 2018. 

## Author
**Chia-Hung Lin** - *clin110*

## Tasks
1. Use camera to identify the location of the Jenga tower.
2. Probe and remove a block from the tower.
3. Put the extracted block back on top of the tower.

## Objectives
1. No human intervention; the robot plays Jenga autonomously.
2. Play at least 6 blocks before collapsing. That is, reach at least 20 levels from the starting 18 levels. (Play a block: to extract a block and put it back on top of the tower.)
3. A 60 second time limit for the robot to play one block.

## ROS Dependency
* **ar_track_alvar** - Track AR tags for the gripper and Jenga tower location. Used for hand-eye calibration and Jenga playing.
* **ur_modern_driver** - Improved driver for the UR5 robot.
* **universal_robot** - Meta package for UR robots. Used for kinematics and urdf.
* **rosserial** - Used to communicate with the Arduino-based end-effector on the robot via USB.

## System Dependency
* **v4l-utils** - Driver for Logitech C920 cameras.

## Package descriptions
* **jenga_ur5_control** - Define waypoints based on tower location identified via AR tags. Generate trajectory to reach a specified block location based on the waypoints.
* **jenga_end_effector** - Provides a-la-carte functionalities for the Arduino based end-effector.
* **jenga_msgs** - Provides control messages for the Arduino-based end-effector. To be used in tandem with *jenga_end_effector* package.
