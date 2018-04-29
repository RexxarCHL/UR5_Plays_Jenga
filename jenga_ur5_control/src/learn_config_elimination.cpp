#include <ros/ros.h>
#include <tf/transform_listener.h>
//#include <tf/transform_broadcaster.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>

#include "jenga_ur5_control/Ur5InverseKinematics.h"

#define SHOULDER_PAN_JOINT  0
#define SHOULDER_LIFT_JOINT 1
#define ELBOW_JOINT         2
#define WRIST_1_JOINT       3
#define WRIST_2_JOINT       4
#define WRIST_3_JOINT       5

#define GRIPPER         0
#define PROBE           1
#define RANGE_FINDER    2

using namespace std

typedef vector<double> Configuration;
typedef pair<Configuration, bool> DataPoint;
typedef vector<DataPoint> DataPoints;

class DataCollector
{
public:
  DataCollector(ros::NodeHandle* nh): nh_(*nh)
  {
    ROS_INFO("Initializing DataCollector");

    ROS_INFO("Initializing subscriber");
    joint_state_subscriber_ = nh_.subscriber<sensor_msgs::JointState>("/joint_states", 10, &jointStateCallback, this);

    ROS_INFO("Initializing service client");
    inverse_kinemaitcs_client_ = nh_.serviceClient<jenga_ur5_control::Ur5InverseKinematics>("ur5_inverse_kinematics");

    ROS_INFO("Waiting for inverse kinematics service to exist and available");
    bool service_exists = inverse_kinemaitcs_client_.waitForExistence(ros::Duration(3.0));
    while (!service_exists) 
    {
      ROS_WARN("Still waiting...");
      ros::spinOnce();
      ros::Duration(1.0).sleep(); // Retry after 1 second
      service_exists = inverse_kinemaitcs_client_.waitForExistence(ros::Duration(1.0));
    }
    ROS_INFO("Service now available");

    ROS_INFO("Initializing action client");
    action_client_ = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(
        "arm_controller/follow_joint_trajectory",
        true);

    ROS_INFO("Waiting for action server to start");
    bool server_exists = action_client_->waitForServer(ros::Duration(3.0));
    while (!server_exists) // Will wait forever
    {
      ROS_WARN("Still waiting...");
      ros::spinOnce();
      ros::Duration(1.0).sleep(); // Retry after 1 second
      server_exists = action_client_->waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("Action server connected!");
  }

  void collectData(int mode, int side, int level, int block)
  {

  }

  DataPoints openExistingData(string file_path)
  {
    DataPoints rv;
    ifstream in_file(file_path);
    if (!in_file) {
      ROS_FATAL("Cannot open file %s", file_path);
      return rv;
    }

    string line;
    while (getline(in_file, line))
    {
      double pan, lift, elbow, wrist_1, wrist_2, wrist_3;
      bool label;
      sscanf(line, "%f %f %f %f %f %f %d", pan, lift, elbow, wrist_1, wrist_2, wrist_3, label);
      Configuration this_config {pan, lift, elbow, wrist_1, wrist_2, wrist_3};
      DataPoints this_data_point(this_config, label);
      rv.push_back(this_data_point);
    }

    in_file.close();

    ROS_INFO("Read %d data points from %s", rv.size(), file_path);
    return rv;
  }

  bool storeCurrentData(string file_path)
  {
    ROS_INFO("Writing %d data points to %s", data_points_.size(), file_path);

    ofstream out_file(file_path);
    if (!out_file) {
      ROS_FATAL("Cannot open file %s", file_path);
      return false;
    }

    for (auto data_point: data_points_)
    {
      Configuration config(data_point.first);
      bool label(data_point.second);

      double pan, lift, elbow, wrist_1, wrist_2, wrist_3;
      pan = config[0]; lift = config[1]; elbow = config[2]; 
      wrist_1 = config[3]; wrist_2 = config[4]; wrist_3 = config[5];

      string line;
      sprintf(line, "%f %f %f %f %f %f %d\n", pan, lift, elbow, wrist_1, wrist_2, wrist_3, label);

      out_file << line;
    }

    out_file.close();

    return true;
  }

  Configuration getCurrentJoints()
  {
    ros::spinOnce(); // Make sure there is no queued joint states callbacks

    // Fix the ordering of positions
    if (joint_state_.name[0] == "elbow_joint")
    {
      std::swap(joint_state_.name[0], joint_state_.name[2]);
      std::swap(joint_state_.position[0], joint_state_.position[2]);
    }

    return joint_state_.position;
  }

  DataPoints data_points_;

private:
  const vector<string> UR_JOINT_NAMES_ {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  const Configuration HOME_CONFIG_ {0, -M_PI/2, 0, -M_PI/2, 0, 0};

  actionlib::SimpleClientGoalState driveToConfiguration(Configuration config)
  {
    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names = UR_JOINT_NAMES_;
    vector<double> zero_vector {0, 0, 0, 0, 0, 0};
    double time_required = 4.0;

    trajectory_msgs::JointTrajectoryPoint joints_current;
    joints_current.positions = getCurrentJoints();
    joints_current.velocities = zero_vector;
    joints_current.time_from_start = ros::Duration(0.0);
    trajectory.points.push_back(joints_current);

    trajectory_msgs::JointTrajectoryPoint joints_up;
    joints_up.positions = HOME_CONFIG_;
    joints_up.velocities = zero_vector;
    joints_up.time_from_start = ros::Duration(time_required);
    trajectory.points.push_back(joints_up);

    trajectory_msgs::JointTrajectoryPoint joints_target;
    joints_target.positions = config;
    joints_target.velocities = zero_vector;
    joints_target.time_from_start = ros::Duration(time_required * 2);
    trajectory.points.push_back(joints_target);

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory;

    return action_client_->sendGoalAndWait(goal); // will wait forever
  }

  /**
   * Change from tf to row major array
   */
  vector<double> transformToRowMajorTransform(tf::Transform transform)
  {
    ROS_INFO("Changing from transform to row major array");

    tf::Matrix3x3 rotation = transform.getBasis();
    tf::Vector3 translation = transform.getOrigin();
    
    vector<double> rv{
        rotation[0].getX(), rotation[0].getY(), rotation[0].getZ(), translation.getX(), 
        rotation[1].getX(), rotation[1].getY(), rotation[1].getZ(), translation.getY(),
        rotation[2].getX(), rotation[2].getY(), rotation[2].getZ(), translation.getZ(),
                       0.0,                0.0,                0.0,                1.0};
    return rv;
  }

  /**
   * Call inverse kinematics service to get possible configurations
   */
  vector<Configuration> getInverseConfigurations(tf::Transform target_transform)
  {
    /* Prepare data for inverse kinematics service */
    jenga_ur5_control::Ur5InverseKinematics srv;
    srv.request.T = transformToRowMajorTransform(target_transform);

    // DEBUG: publish this frame
    tf_broadcaster_.sendTransform(
        tf::StampedTransform(target_transform, ros::Time::now(), "base_link", "target_transform") );

    /* Call inverse kinematics service */
    vector<Configuration> rv;

    ROS_INFO("Calling inverse kinematics service");
    bool srv_result = inverse_kinemaitcs_client_.call(srv);
    if (!srv_result)
    {
      ROS_WARN("Service failed!");
      return rv; // Empty vector
    }

    if(!srv.response.num_sols)
    {
      ROS_ERROR("No solution found!");
      return rv; // Empty vector
    }

    // DEBUG: print out the results
    ROS_INFO("Inverse kinematics results (%d solutions found):", srv.response.num_sols);
    //debugPrintJoints(srv.response.q_sols); // this should work
    
    /* Store the results in a more easily accessible vector format */
    for (int sol = 0; sol < srv.response.num_sols; ++sol)
    {
      Configuration this_config;
      for (int i = 0; i < 6; ++i)
      {
        double q = srv.response.q_sols[sol*6 + i];
        // Results from inverse kinematics are in the range of [0, 2PI). (see ur_kinematics/ur_kin.h)
        // Remap to [-PI, PI] for safty purposes when operating with actual robot
        if (q > M_PI)
          q -= 2 * M_PI;
        this_config.push_back(q);
      }
      rv.push_back(this_config);
    }

    return rv;
  }

  /**
   * Wait and return the transfrom by the input name
   */
  tf::Transform retrieveTransform(string frame_name)
  {
    /* TODO: Return different frames for gripper, probe and laser range finder */

    ROS_INFO("Retreiving frame: %s", frame_name.c_str());
    
    // Wait for the frame to spawn
    bool frame_exists = tf_listener_.waitForTransform("base_link", frame_name, ros::Time(), ros::Duration(1.0));
    while(!frame_exists)
    {
      ROS_WARN("Frame \"%s\" does not exist; retrying...", frame_name.c_str());
      ros::spinOnce();
      ros::Duration(1.0).sleep();
      frame_exists = tf_listener_.waitForTransform("base_link", frame_name, ros::Time(), ros::Duration(1.0));
    }

    // Read the frame
    tf::StampedTransform tf_stamped;
    tf_listener_.lookupTransform("base_link", frame_name, ros::Time(), tf_stamped);
    // Change from stamped transform to normal transform
    tf::Transform transform(tf_stamped.getRotation(), tf_stamped.getOrigin());

    return transform;
  }

  /** 
   * Get a ee_link transformation for the specified block, depending on input action
   */
  tf::Transform getTransformFor(int mode, int side, int level, int block)
  {
    tf::Transform tf_block = targetBlockToTargetTransform(side, level, block);

    switch(mode)
    {
      case GRIPPER:
          ROS_INFO("Getting transform for gripper at side%d, level%d, block %d", side, level, block);
          tf_block = compensateEELinkToGripper(tf_block);
          break;
      case PROBE:
          ROS_INFO("Getting transform for probe at side%d, level%d, block %d", side, level, block);
          tf_block = compensateEELinkToProbe(tf_block);
          break;
      case RANGE_FINDER:
          ROS_INFO("Getting transform for range finder at side%d, level%d, block %d", side, level, block);
          tf_block = compensateEELinkToRangeFinder(tf_block);
          break;
    }

    return tf_block;
  }

  /**
   * Compensate ee_link -> tool0 -> tool_{gripper/range_finder/probe} 
   */
  tf::Transform compensateEELinkToTool0(tf::Transform transform)
  {
    tf::Transform ee_link_to_tool0( tf::Quaternion(-0.5, 0.5, -0.5, 0.5) ); 
    transform = transform * ee_link_to_tool0.inverse();

    return transform;
  }
  tf::Transform compensateEELinkToGripper(tf::Transform transform)
  {
    tf::Transform ee_link_to_tool0( tf::Quaternion(-0.5, 0.5, -0.5, 0.5) ); 
    tf::Transform tool0_to_tool_gripper( tf::Quaternion::getIdentity(), tf::Vector3(0, 0, 0.072646) );

    tf::Transform ee_link_to_tool_gripper = ee_link_to_tool0 * tool0_to_tool_gripper;

    transform = transform * ee_link_to_tool_gripper.inverse();

    return transform;
  }
  tf::Transform compensateEELinkToRangeFinder(tf::Transform transform)
  {
    tf::Transform ee_link_to_tool0( tf::Quaternion(-0.5, 0.5, -0.5, 0.5) ); 
    tf::Transform tool0_to_tool_range_finder( tf::Quaternion::getIdentity(), tf::Vector3(0, -0.02942, 0.059234) );

    tf::Transform ee_link_to_tool_range_finder = ee_link_to_tool0 * tool0_to_tool_range_finder;

    transform = transform * ee_link_to_tool_range_finder.inverse();

    return transform;
  }
  tf::Transform compensateEELinkToProbe(tf::Transform transform)
  {
    tf::Transform ee_link_to_tool0( tf::Quaternion(-0.5, 0.5, -0.5, 0.5) ); 
    tf::Transform tool0_to_tool_probe( tf::Quaternion(0.0, M_PI/4, 0.0, M_PI/4), tf::Vector3(0.0795, 0, 0.026146) );

    tf::Transform ee_link_to_tool_probe = ee_link_to_tool0 * tool0_to_tool_probe;

    transform = transform * ee_link_to_tool_probe.inverse();

    return transform;
  }

  bool askForUserLabel()
  {
    ROS_WARN("How is this configuration?");
    int result;
    cin >> result;

    char ch;
    while (cin.readsome(&ch, 1) != 0)
         ; // do nothing

    return (bool)result;
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joints);

  ros::NodeHandle nh_;
  ros::Subscriber joint_state_subscriber_;
  ros::ServiceClient inverse_kinematics_client_;

  //tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* action_client_;

  sensor_msgs::JointState joint_state_;
}