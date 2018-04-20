#include <ros/ros.h>
#include "jenga_ur5_control/Ur5ForwardKinematics.h"
#include "jenga_ur5_control/Ur5ForwardKinematicsAll.h"
#include "jenga_ur5_control/Ur5InverseKinematics.h"

#include <ur_kinematics/ur_kin.h>

bool ur5ForwardKinematicsCallback(
    jenga_ur5_control::Ur5ForwardKinematics::Request &req,
    jenga_ur5_control::Ur5ForwardKinematics::Response &res)
{
  ROS_INFO("ur5_fwd_kin called");
  if (req.q.size() != 6)
  {
    ROS_ERROR("The size of q must be 6");
    return false;
  }

  res.T.resize(16);
  ur_kinematics::forward( req.q.data(), res.T.data() );

  return true;
}

bool ur5ForwardKinematicsAllCallback(
    jenga_ur5_control::Ur5ForwardKinematicsAll::Request &req,
    jenga_ur5_control::Ur5ForwardKinematicsAll::Response &res)
{
  ROS_INFO("ur5_fwd_kin_all called");
  if (req.q.size() != 6)
  {
    ROS_ERROR("The size of q must be 6");
    return false;
  }

  res.T1.resize(16);
  res.T2.resize(16);
  res.T3.resize(16);
  res.T4.resize(16);
  res.T5.resize(16);
  res.T6.resize(16);

  ur_kinematics::forward_all(
      req.q.data(), 
      res.T1.data(), res.T2.data(), res.T3.data(), res.T4.data(), res.T5.data(), res.T6.data() );

  return true;
}

bool ur5InverseKinematicsCallback(
    jenga_ur5_control::Ur5InverseKinematics::Request &req,
    jenga_ur5_control::Ur5InverseKinematics::Response &res)
{
  ROS_INFO("ur5_inv_kin called");
  if (req.T.size() != 16)
  {
    ROS_ERROR("The size of T must be 16");
    return false;
  }
  
  res.q_sols.resize(8 * 6);

  res.num_sols = ur_kinematics::inverse(req.T.data(), res.q_sols.data(), 0);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur_kinematics_server");
  ros::NodeHandle nh;

  ros::ServiceServer fwdKinService = nh.advertiseService("ur5_forward_kinematics", ur5ForwardKinematicsCallback);
  ros::ServiceServer fwdKinAllService = 
      nh.advertiseService("ur5_forward_kinematics_all", ur5ForwardKinematicsAllCallback);
  ros::ServiceServer invKinService = nh.advertiseService("ur5_inverse_kinematics", ur5InverseKinematicsCallback);

  ROS_INFO("UR5 kinematics service ready");
  ros::spin();

  return 0;
}