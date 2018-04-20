/**
 * Trajectory Control class
 * Provides Jenga specific trajectory generation and arm control for the UR5.
 * Author: Chia-Hung Lin (clin110[AT]jhu[DOT]edu)
 * Date Created: 04/08/2018
 */
#ifndef JENGA_TRAJ_CTRL_H
#define JENGA_TRAJ_CTRL_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>
#include <jenga_msgs/EndEffectorControl.h>
#include <jenga_msgs/EndEffectorFeedback.h>
#include <jenga_msgs/Probe.h>

#include "jenga_ur5_control/Ur5InverseKinematics.h"
#include "jenga_ur5_control/JengaTarget.h"


/**
 * AR paper orientation
 * 1st level on the ground is oriented 0-2  
 *                          __
 *                         |  |<- Robot Base
 *               side1     |__|
 *               1 0 -1
 *               _ _ _
 *              | | | |
 *              | | | |
 *              |_|_|_|
 * s      ____   _____   ____     s
 * i  -1 |____| |_1st_| |____| -1 i
 * d   0 |____| |_____| |____|  0 d
 * e   1 |____| |_lvl_| |____|  1 e
 * 0             _ _ _            2
 *              | | | |
 *              | | | |
 *              |_|_|_|
 *               1 0 -1
 *               side 3
 */
#define GRIPPER         0
#define PROBE           1
#define RANGE_FINDER    2 

#define GRIPPER_OPEN_WIDE    0
#define GRIPPER_OPEN_NARROW  1
#define GRIPPER_CLOSE_WIDE   2
#define GRIPPER_CLOSE_NARROW 3

#define SHOULDER_PAN_JOINT  0
#define SHOULDER_LIFT_JOINT 1
#define ELBOW_JOINT         2
#define WRIST_1_JOINT       3
#define WRIST_2_JOINT       4
#define WRIST_3_JOINT       5

class TrajCtrl // short for Trajectory Control
{
public:
  typedef std::vector<double> Configuration;

  TrajCtrl(ros::NodeHandle* nh);
  
  /**
   * Drive the arm via proper trajectory to the target block location.
   *
   * Input:
   *   side : The side indicated on the AR tag paper. Can take on 0, 1, 2, or 3.
   *   level: The level in the Jenga tower the target block is on.
   *   block: The block number indicated on the AR tag paper. Can take on -1, 0, or 1.
   *
   * Return: Action status code. (See: http://docs.ros.org/jade/api/actionlib_msgs/html/msg/GoalStatus.html)
   */
  actionlib::SimpleClientGoalState driveArmToJengaBlock(int side, int level, int block);
  // Return internal joint state
  sensor_msgs::JointState getCurrentJointState();

  void debugTestFunctions();
  void debugTestFlow();

  /*******************************************************************
   *                     JENGA PLAYING FUNCTIONS                     *
   *******************************************************************/
  /**
   * Move to a specified location >>from current configuration<<<.
   * These movement actions are assumed to be called when the robot is in home position
   *
   * Input:
   *   action: GRIPPER, PROBE, or RANGE_FINDER
   *   side  : The side indicated on the AR tag paper. Can take on 0, 1, 2, or 3.
   *   level : The level in the Jenga tower the target block is on.
   *   block : The block number indicated on the AR tag paper. Can take on -1, 0, or 1.
   * Return: Action status code. (See: http://docs.ros.org/jade/api/actionlib_msgs/html/msg/GoalStatus.html)
   */
  actionlib::SimpleClientGoalState moveToActionPosition(int action, int side, int level, int block);
  actionlib::SimpleClientGoalState moveToHomePosition(int side);
  //actionlib::SimpleClientGoalState moveToGripChangePosition();
  actionlib::SimpleClientGoalState moveToPlaceBlockPosition(); // Move to a spot above an empty slot

  /**
   * Execute a specific action >>>from current configuration<<<.
   *
   * Return: Action status code. (See: http://docs.ros.org/jade/api/actionlib_msgs/html/msg/GoalStatus.html)
   */
  actionlib::SimpleClientGoalState executeProbingAction(); // Move in and push 25mm
  actionlib::SimpleClientGoalState executeGrippingAction(int mode); // Move in and close gripper
  actionlib::SimpleClientGoalState executeRangeFindingAction(); // Move left and right
  actionlib::SimpleClientGoalState executeGripChangeAction();
  actionlib::SimpleClientGoalState executePlaceBlockAction();

private:
  const std::vector<std::string> UR_JOINT_NAMES_ {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  const Configuration HOME_CONFIG_ {0, -M_PI/2, 0, -M_PI/2, 0, 0};

  ros::NodeHandle nh_;
  ros::Publisher tool_command_publisher_;
  ros::Subscriber tool_feedback_subscriber_;
  ros::Subscriber tool_range_subscriber_;
  ros::Subscriber tool_probe_subscriber_;
  ros::Subscriber jenga_target_subscriber_;
  ros::Subscriber joint_state_subscriber_;
  ros::ServiceClient inverse_kinemaitcs_client_;
  
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;
  
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* action_client_;
  
  sensor_msgs::JointState joint_state_;
  //jenga_msgs::EndEffectorControl tool_command_;

  // Game state
  int current_level_; // The current top level count
  std::vector<int> top_status_; // The current block occupancy on top of the tower
  bool top_orientation_; // The current correct orientation to place a block
  int checkGameState(); // Check and update game state. Return first empty block slot


  bool is_busy_;
  bool is_probing_;

  // Tool related
  bool tool_feedback_flag_; // Flag to indicate if there is a new message available
  int tool_feedback_code_;  // Feedback from the tool
  void publishToolCommand(int command_code);
  bool blockUntilToolFeedback(int feedback_code); // Block until a feedback from the tool is received
  void feedbackCallback(const jenga_msgs::EndEffectorFeedback::ConstPtr& msg);
  void probeCallback(const jenga_msgs::Probe::ConstPtr& msg);
  void rangeCallback(const sensor_msgs::Range::ConstPtr& msg);

  // Initialize subscribers and link their callbacks 
  void initializeSubscriber();
  // Initialize publisher
  void initializePublisher();
  // Initialize connection to action server on follow_joint_trajectory
  void initializeActionClient();
  // Initialize inverse kinematics service client
  void initializeServiceClient();

  /**
   * Callback function for receiving a target block. Move the robot arm to the target location.
   * Returns immediate and do nothing if there is an action in progress
   */
  void jengaTargetCallback(const jenga_ur5_control::JengaTarget::ConstPtr& target_block);
  // Callback function for when an update for joint state is available. Update internal joint state.
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joints);

  /*******************************************************************
   *                        INVERSE KINEMATICS                       *
   *******************************************************************/
  /**
   * Change a transform to row major array for inverse kinematics.
   *
   * Input:
   *   transform: A tf transform.
   *
   * Return: Size 16 vector. Row major equivalent of the input argument. 
   */
  std::vector<double> transformToRowMajorTransform(tf::Transform transform);

  /** 
   * Call inverse kinematics service to get possible configurations for the >>>tool0<<< frame on the arm to reach
   * input transform.
   *
   * Input:
   *   targetTransform: Desired tf transform for the arm to reach.
   *
   * Return: A vector containing possible configurations for the target transform. Size up to 8.
   */
  std::vector<Configuration> getInverseConfigurations(tf::Transform target_transform);

  /**
   * Eliminate some configurations returned by inverse kinematics that are not preferred for Jenga playing.
   * 
   * Input:
   *   configurations: A vector of configurations to eliminate.
   * Return: A vector of eliminated configuration. At least size 1.
   */
  Configuration eliminateConfigurations(std::vector<Configuration> configurations);

  /**
   * Pick the configuration that requires least effort from start configuration. 
   * 
   * Input:
   *   possible_configurations: A vector of configurations to eliminate.
   *   start_configuration    : The starting configuration.
   * Return: The least difference configuration compared to start_configuration.
   */
  Configuration pickMinimumEffortConfiguration(
      std::vector<Configuration> possible_configurations, 
      Configuration start_configuration);


  /*******************************************************************
   *                         TF MANIPULATION                         *
   *******************************************************************/
  /**
   * Get a ee_link transformation for the specified block, depending on input action
   * 
   * Input:
   *   mode : PROBE, RANGE_FINDER, or GRIPPER
   *   side : The side indicated on the AR tag paper. Can take on 0, 1, 2, or 3.
   *   level: The level in the Jenga tower the target block is on.
   *   block: The block number indicated on the AR tag paper. Can take on -1, 0, or 1.
   * Return: Transform of >>>ee_link<<< in probing position for this block
   */
  tf::Transform getTransformFor(int mode, int side, int level, int block);

  /**
   * Wait and return the transform (wrt base_link) by the name of the input argument
   *
   * Input:
   *   frame_name: The name of the frame to look up.
   *
   * Return: The corresponding tf transform.
   */

  tf::Transform retrieveTransform(std::string frame_name);

  /**
   * Get a tf transform location from the input block for the robot to move to.
   * 
   * Input:
   *   side : The side indicated on the AR tag paper. Can take on 0, 1, 2, or 3.
   *   level: The level in the Jenga tower the target block is on.
   *   block: The block number indicated on the AR tag paper. Can take on -1, 0, or 1.
   *
   * Return: A tf transform corresponding to the input block location.
   */
  tf::Transform targetBlockToTargetTransform(int side, int level, int block);

  /**
   * Waypoints are specified corresponding to tool0 frame. 
   * Compensate ee_link -> tool0 -> tool_{gripper/range_finder/probe} by multiplying appropriate transformation.
   * 
   * Input:
   *   transform: Transform specified by tool0.
   * Return: Appropriately compensated transform.
   */
  tf::Transform compensateEELinkToTool0(tf::Transform transform);
  tf::Transform compensateEELinkToGripper(tf::Transform transform);
  tf::Transform compensateEELinkToRangeFinder(tf::Transform transform);
  tf::Transform compensateEELinkToProbe(tf::Transform transform);
  
  /*******************************************************************
   *                      TRAJECTORY GENERATION                      *
   *******************************************************************/
  /**
   * Send trajectory goal to the action server and block until execution returned.
   *
   * Input: 
   *   trajectory_goal: The trajectory to execute.
   * Return: Action status code. (See: http://docs.ros.org/jade/api/actionlib_msgs/html/msg/GoalStatus.html)
   */
  actionlib::SimpleClientGoalState executeTrajectoryGoal(control_msgs::FollowJointTrajectoryGoal trajectory_goal);

  /**
   * Generate a trajectory goal object that drive the arm from current position to input target_transform.
   *
   * Input:
   *   side     : The side of the tower the target transform is on. //TODO: maybe auto detect base on target distance?
   *   tf_target: Desired final >>>ee_link<<< transformation.
   * Return: A trajectory goal object ready to be executed.
   */
  control_msgs::FollowJointTrajectoryGoal generateTrajectory(int side, tf::Transform tf_target);

  /**
   * Generate a trajectory back to "up" position.
   *
   * Input:
   *   side: The side of the tower where the robot is currently on. //TODO: may be auto detect?
   * Return: A trajectory object ready to be executed.
   */
  control_msgs::FollowJointTrajectoryGoal generateHomingTrajectory(int side);

  /**
   * Check the current configuration of the robot to see if it matches home configuration
   * Issues a warning if it is not in home configuration.
   */
  void checkRobotInHomeConfig();

  void debugPrintJoints(Configuration joints);
  void debugBreak();
}; // class TrajCtrl

#endif // JENGA_TRAJ_CTRL_H
