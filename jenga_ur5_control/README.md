# UR5 Plays Jenga - jenga_ur5_control

Provides trajectory generation and action execution for the AI player to interact with the robot. AI player needs only to designate one or more jenga blocks as target, and this control node will automatically try to play the block(s). 

## Author
**Chia-Hung Lin** - *clin110*

## ROS Dependency
* jenga_msgs - Messages needed to receive jenga target blocks and send results.
* industrial_core - Provides actionlib follow_joint_trajectory to control the robot along joint trajectory points. 
* universal_robot - Meta package for UR robots.
* ur_modern_driver - A drop-in alternative for the dated ur_driver in the universal_robot package.