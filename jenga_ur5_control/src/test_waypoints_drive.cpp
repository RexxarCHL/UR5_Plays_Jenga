#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>

#include "jenga_ur5_control/Ur5InverseKinematics.h"

std::vector<std::string> g_ur_joint_names {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
sensor_msgs::JointState g_joint_state;

#define SHOULDER_PAN_JOINT 0
#define SHOULDER_LIFT_JOINT 1
#define ELBOW_JOINT 2
#define WRIST_1_JOINT 3
#define WRIST_2_JOINT 4
#define WRIST_3_JOINT 5

// Update joint state
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint)
{
  //ROS_INFO("joint state updated");
  std::cout << std::flush;
  g_joint_state = *joint;
}

control_msgs::FollowJointTrajectoryGoal getGoal(const trajectory_msgs::JointTrajectory trajectory)
{
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  //action_client.sendGoal(trajectory, actionDoneCallback);
  //action_client.sendGoal(goal);
  return goal;
}
/*
void actionDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const control_msgs::FollowJointTrajectoryResultConstPtr& result){
  //do something
}
*/

// Function 'inverse' in ur_kinematics package requires 4x4 end effector pose in row-major ordering
std::vector<double> transformToRowMajorTransform(tf::Transform transform)
{
  tf::Matrix3x3 rotation = transform.getBasis();
  tf::Vector3 translation = transform.getOrigin();

  std::vector<double> rv {
      rotation[0].getX(), rotation[0].getY(), rotation[0].getZ(), translation.getX(), 
      rotation[1].getX(), rotation[1].getY(), rotation[1].getZ(), translation.getY(),
      rotation[2].getX(), rotation[2].getY(), rotation[2].getZ(), translation.getZ(),
                     0.0,                0.0,                0.0,                1.0};
  
  ROS_INFO("Changing from stamped transform to row major transform");
  for (auto v: rv) 
    std::cout << v << "," << std::flush;
  std::cout << std::endl;

  return rv;
}

std::vector<std::vector<double>> cherrypickConfigurations(std::vector<std::vector<double>> configurations)
{
  ROS_INFO("cherry picking configurations...");
  std::vector<std::vector<double>> rv;
  
  for (auto config: configurations) 
  {
    ROS_INFO("This configuration:");
    for (auto q: config) 
    {
      std::cout << q << " " << std::flush;
    }
    std::cout << std::endl;

    if (config[SHOULDER_LIFT_JOINT] > -M_PI/8.0 || config[SHOULDER_LIFT_JOINT] < -7.0/8.0*M_PI) 
    {
      ROS_INFO("--REJECTED: shoulder_lift_joint");
      continue;
    }

    if (config[SHOULDER_PAN_JOINT] > 1.0 || config[SHOULDER_PAN_JOINT] < -2.1) 
    {
      ROS_INFO("--REJECTED: shoulder_pan_joint");
      continue;
    } 

    if (config[WRIST_1_JOINT] > 0.3)
    {
      ROS_INFO("--REJECTED: wrist_1_joint");
      continue;
    }

    // If reached here, this config is probably good
    ROS_INFO("--OK!");
    rv.push_back(config);
  }

  ROS_INFO("Found %lu probably good solutions", rv.size());
  if (!rv.size())
    ROS_WARN("NO GOOD SOLUTIONS FOUND!");

  return rv;
}

int main(int argc, char** argv)
{
  /* Initialize ros node and handle */
  ros::init(argc, argv, "test_waypoints_drive");
  ros::NodeHandle nh;

  ros::Subscriber joint_state_subscriber =
      nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, jointStateCallback);
  ros::spinOnce();
  //ros::Subscriber trajectory_goal_subscriber =
  //    nh.subscribe<trajectory_msgs::FollowJointTrajectoryGoal>("trajectory_goal", 10, trajectoryGoalCallback);

  /* Initiate client of the arm server */
  // Create action client for follow_joint_trajectory
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> 
      action_client("arm_controller/follow_joint_trajectory", true);

  // Attempt to connect to server
  ROS_INFO("Waiting for action server to start");
  bool server_exists = action_client.waitForServer(ros::Duration(1.0));
  while (!server_exists) 
  {
    ROS_WARN("Still waiting...");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    server_exists = action_client.waitForServer(ros::Duration(1.0));
  }
  ROS_INFO("Action server connected");


  /* Get transformation to a target frame */
  tf::TransformListener frame_listener;
  std::string target_frame("roadmap_side3");
  ROS_INFO("Testing for %s", target_frame.c_str());
  // Wait for the frame to spawn
  bool frame_exists = frame_listener.waitForTransform("base_link", target_frame, 
                                                      ros::Time(), ros::Duration(1.0));
  while (!frame_exists) 
  {
    ROS_WARN("Frame \"%s\" does not exist; retrying...", target_frame.c_str());
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    frame_exists = frame_listener.waitForTransform("base_link", target_frame, ros::Time(), ros::Duration(1.0));
  }
  // Read the frame
  tf::StampedTransform stampedTransform;
  frame_listener.lookupTransform("base_link", target_frame, ros::Time(0), stampedTransform);
  // Change from stamped transform to normal transform
  tf::Transform transform(stampedTransform.getRotation(), stampedTransform.getOrigin());
  // Account for the difference between ee_link and tool0
  tf::Transform eeLinkToTool0(tf::Quaternion(-0.500, 0.500, -0.500, 0.500)); 
  transform = transform * eeLinkToTool0.inverse();
  
  /* Get the possible configurations to target frame by inverse kinematics */
  ros::ServiceClient inverse_kinematics_client = // Create a service client for inverse kinematics
      nh.serviceClient<jenga_ur5_control::Ur5InverseKinematics>("ur5_inverse_kinematics");
  // Initiate service request
  jenga_ur5_control::Ur5InverseKinematics srv;
  srv.request.T = transformToRowMajorTransform(transform);
  // Call service to get 8 inverse kinematics configurations for this transformation
  bool srv_result = inverse_kinematics_client.call(srv);
  if(!srv_result) 
  {
    ROS_WARN("service failed");
    return 1; // Service call failed; clean up?
  }

  if (!srv.response.num_sols) 
  {
    ROS_WARN("NO SOLUTION FOUND!!");
    ros::spin();
  }

  ROS_INFO("inv kin results (%d solutions found):", srv.response.num_sols);
  for (auto q_sol: srv.response.q_sols) 
  {
    std::cout << q_sol << "," << std::flush;
  }
  std::cout << std::endl;

  // Service call success; store it in a more obvious variable
  std::vector<std::vector<double>> configurations; // A vector of possible configurations
  /*
  auto q_sols_begin = srv.response.q_sols.begin();
  for(int i = 0; i < srv.response.num_sols; ++i){
    //std::vector<double> config( q_sols_begin + i*6, q_sols_begin + (i + 1)*6);
    //ROS_INFO("size of config=%lu", config.size());
    //configurations.push_back(config);
    configurations.push_back(std::vector<double>(q_sols_begin + i*6, q_sols_begin + (i + 1)*6));
  }
  */
  for (int sol = 0; sol < srv.response.num_sols; ++sol) 
  {
    std::vector<double> thisConfig;
    for (int i = 0; i < 6; ++i) 
    {
      double q = srv.response.q_sols[sol*6 + i];
      std::cout << q << "," << std::flush;
      if (q > M_PI)
        q -= 2 * M_PI;
      thisConfig.push_back(q);
    }
    std::cout << std::endl;
    configurations.push_back(thisConfig);
  }

  configurations = cherrypickConfigurations(configurations);

  /* Drive the arm to each and every possible configurations */
  std::vector<double> initial_joint_state {0, -M_PI/2, 0, -M_PI/2, 0, 0}; // Up position
  double time_required = 3.0;
  //std::vector<double> zero_vector (6, 0); // A vector of six 0's
  trajectory_msgs::JointTrajectoryPoint initial_joints;
  initial_joints.positions = initial_joint_state;
  g_joint_state.position = initial_joint_state; // In simulation the it starts at home position; drive it to up first
  //ROS_INFO("size of middle point=%lu", initial_joints.positions.size());
  //initial_joints.time_from_start = ros::Duration(time_required);
  for (auto it = configurations.begin(); it != configurations.end(); ++it) 
  {
    trajectory_msgs::JointTrajectory trajectory; // Initialize new trajectory
    trajectory.joint_names = g_ur_joint_names;

    // Start from current joint positions
    trajectory_msgs::JointTrajectoryPoint current_joints;
    current_joints.positions = g_joint_state.position; 
    ROS_INFO("current_joints:");
    for (auto q: current_joints.positions)
      std::cout << q << " " << std::flush;
    std::cout << std::endl;

    current_joints.time_from_start = ros::Duration(time_required);
    trajectory.points.push_back(current_joints); // Push to points vector

    // Return to initial state first
    //initial_joints.time_from_start = ros::Duration(time_required * 2);
    //trajectory.points.push_back(initial_joints); 

    // Go to next configuration
    trajectory_msgs::JointTrajectoryPoint next_joints;
    std::vector<double> configuration = *it;
    next_joints.positions = configuration;
    ROS_INFO("next_joints:");
    for (auto q: next_joints.positions)
      std::cout << q << " " << std::flush;
    std::cout << std::endl;
    next_joints.time_from_start = ros::Duration(time_required * 3);
    trajectory.points.push_back(next_joints);

    ROS_INFO("Sending goal to action server");
    actionlib::SimpleClientGoalState status = action_client.sendGoalAndWait(getGoal(trajectory));
    ROS_INFO("Action status: %s", status.toString().c_str());

    ROS_INFO("Waiting 10 sec before next action");
    ros::spinOnce();
    ros::Duration(10.0).sleep();
    //action_client.cancelGoal();
  }

  return 0;
}
