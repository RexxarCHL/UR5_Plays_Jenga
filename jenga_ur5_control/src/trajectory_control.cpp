/**
 * Trajectory Control class
 * Provides Jenga specific trajectory generation and arm control for the UR5.
 * Author: Chia-Hung Lin (clin110[AT]jhu[DOT]edu)
 * Date Created: 04/08/2018
 */
#include "jenga_ur5_control/trajectory_control.h"

/**
 * Constructor
 */
TrajCtrl::TrajCtrl(ros::NodeHandle* nh): nh_(*nh)
{
  ROS_INFO("Instantiating Trajectory Control object");

  current_level_ = 18;
  //top_orientation_ = true;
  top_orientation_ = false;
  top_status_ = std::vector<int> {1, 1, 1};

  //joint_state_.position = std::vector<double>(0.0, 6);

  is_busy_ = false;
  is_probing_ = false;

  initializeSubscriber();
  initializeServiceClient();
  initializeActionClient();

  ros::spinOnce(); // ensure callbacks work properly
}

/**
 * Initialize subscribers and link their callbacks 
 */
void TrajCtrl::initializeSubscriber()
{
  ROS_INFO("Initializing subscribers");

  jenga_target_subscriber_ = // Queue size of 1 to prevent extra actions from queuing
      nh_.subscribe<jenga_ur5_control::JengaTarget>("/jenga_target", 1, &TrajCtrl::jengaTargetCallback, this);
  joint_state_subscriber_ = 
      nh_.subscribe<sensor_msgs::JointState>("/joint_states", 10, &TrajCtrl::jointStateCallback, this);

  // Tool related
  tool_feedback_subscriber_ = 
      nh_.subscribe<jenga_msgs::EndEffectorFeedback>("/tool/feedback", 1, &TrajCtrl::feedbackCallback, this);
  tool_range_subscriber_ =
      nh_.subscribe<sensor_msgs::Range>("/tool/range", 10, &TrajCtrl::rangeCallback, this);
  tool_probe_subscriber_ = 
      nh_.subscribe<jenga_msgs::Probe>("/tool/probe", 3, &TrajCtrl::probeCallback, this);
}
/**
 * Initialize publisher
 */
void TrajCtrl::initializePublisher()
{
  ROS_INFO("Initializing publishers");

  tool_command_publisher_ = nh_.advertise<jenga_msgs::EndEffectorControl>("/tool/command", 3);
}

/**
 * Initialize connection to action server on follow_joint_trajectory
 */
void TrajCtrl::initializeActionClient()
{
  ROS_INFO("Initializing action client");

  // HACK! Account for action server namespace difference between gazebo and actual robot 
  bool is_simulation;
  nh_.getParam("is_simulation", is_simulation);
  std::string action_server_name = is_simulation? "arm_controller/follow_joint_trajectory":"follow_joint_trajectory";
  ROS_INFO("using namespace %s", action_server_name.c_str());

  action_client_ = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(
      //"follow_joint_trajectory", // For actual robot
      //"arm_controller/follow_joint_trajectory", // For simulation
      action_server_name,
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

/**
 * Initialize inverse kinematics service client
 */
void TrajCtrl::initializeServiceClient()
{
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
}

/**
 * Return internal joint state
 */
sensor_msgs::JointState TrajCtrl::getCurrentJointState()
{
  ros::spinOnce(); // Make sure there is no queued joint states callbacks

  // Fix the ordering of positions
  if (joint_state_.name[0] == "elbow_joint")
  {
    std::swap(joint_state_.name[0], joint_state_.name[2]);
    std::swap(joint_state_.position[0], joint_state_.position[2]);
  }

  for(int i = 0; i < 6; ++i)
    std::cout << joint_state_.name[i] << ": " << joint_state_.position[i] << std::endl;
  return joint_state_;
}

/**
 * Update internal joint state
 */
void TrajCtrl::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joints)
{
  //ROS_WARN("Update joint state");
  joint_state_ = *joints;
}

/**
 * Move the robot arm to the target location. Skips execution if there is an action in progress
 */
void TrajCtrl::jengaTargetCallback(const jenga_ur5_control::JengaTarget::ConstPtr& target_block)
{
  /* Received candidate blocks */ 
  // TODO: Modify message to receive blocks instead of just one block
  ROS_INFO("Received jenga target block: side%d, level%d, block#%d", 
           target_block->side, target_block->level, target_block->block);
  if(is_busy_) // Block arm driving action from executing until the previous action is done
    return;

  is_busy_ = true; // Lock future action requests

  /* Pick a block */
  // TODO: After modifying the message, pick a block sequentially from the message
  int side = target_block->side;
  int level = target_block->level;
  int block = target_block->block;

  //TODO: Get everything under here to be an action send to another node?

  /* Move arm to probing position */
  actionlib::SimpleClientGoalState status = moveToActionPosition(PROBE, side, level, block);

  /* Probe the block */
  status = executeProbingAction();
  // TODO: The subroutine monitoring the force readings will have to somehow signal here
  //       for decision to continue or to try next block

  /* Return to up position */
  status = moveToHomePosition(side);

  // TODO: branch according to probing results

  /* Move arm to range finding position on the other side */
  int other_side = (side + 2) % 4;
  status = moveToActionPosition(RANGE_FINDER, other_side, level, block);

  /* Use range finder to find center of the block */
  status = executeRangeFindingAction();

  /* Find center of the block and compensate accordingly */
  // TODO
  // tf::Transform compensation_transform = ...

  /* Move arm to gripping position */
  status = moveToActionPosition(GRIPPER, other_side, level, block);

  /* Grip the block and pull it out */
  status = executeGrippingAction(GRIPPER_CLOSE_NARROW); // Gripping action will auto return to home position

  /* Return to up position */
  status = moveToHomePosition(other_side);

  /* Move arm to grip change position */
 // status = moveToGripChangePosition();
  //ROS_INFO("Moving to grip change position returned with result %s", status.toString().c_str() );

  /* Change the grip fron short side to long side */
  status = executeGripChangeAction();
  ROS_INFO("Grip changing returned with result %s", status.toString().c_str() );

  /* Return to up position */
  status = moveToHomePosition(other_side);

  /* Move to block placing position */
  status = moveToPlaceBlockPosition();

  /* Place the block */
  status = executePlaceBlockAction();

  /* Return to up position */
  status = moveToHomePosition(other_side);

  /* Voila! We are done playing this block */
  // TODO: report what block is played
  is_busy_ = false; // Release the lock
}

/**
 * Move the arm to the indicated location
 */
actionlib::SimpleClientGoalState TrajCtrl::driveArmToJengaBlock(int side, int level, int block)
{
  ROS_INFO("Drive arm to target block: side%d, level%d, block#%d", side, level, block);

  /* Get the tf transformation to this block location */
  tf::Transform tf_block = targetBlockToTargetTransform(side, level, block);

  /* Lookup transform on the route */
  std::string frame_name_above("roadmap_above" + std::to_string(side) );
  std::string frame_name_side("roadmap_side" + std::to_string(side) );

  tf::Transform tf_above, tf_side;
  tf_above = retrieveTransform(frame_name_above);
  tf_side = retrieveTransform(frame_name_side);

  // DEBUG: show the frames
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(tf_above, ros::Time::now(), "base_link", "tf_above_ee_link"));
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(tf_side, ros::Time::now(), "base_link", "tf_side_ee_link"));

  /* Call inverse kinematics to get possible configurations */
  // TODO: Pre-calculate the waypoint configurations?
  std::vector<TrajCtrl::Configuration> inv_configs_above, inv_configs_side, inv_configs_block; // Inverse configurations
  inv_configs_above = getInverseConfigurations(tf_above);
  inv_configs_side = getInverseConfigurations(tf_side);
  inv_configs_block = getInverseConfigurations(tf_block);

  /* Eliminate configurations to only one per waypoint */
  TrajCtrl::Configuration config_above, config_side, config_block;
  // Eliminate configurations for roadmap_above using heuristic method
  config_above = eliminateConfigurations(inv_configs_above);

  // Eliminate configurations for roadmap_side and roadmap_block by least difference to its predecessor
  // The robot moves in this order: roadmap_above -> roadmap_side -> roadmap_block
  config_side = pickMinimumEffortConfiguration(inv_configs_side, config_above);
  config_block = pickMinimumEffortConfiguration(inv_configs_block, config_side);

  /* Drive the arm to the waypoints sequentially */
  ROS_INFO("Initializing new trajectory");
  trajectory_msgs::JointTrajectory trajectory; // Initialize new trajectory
  trajectory.joint_names = UR_JOINT_NAMES_;
  std::vector<double> zero_vector{0, 0, 0, 0, 0, 0};
  double time_required = 4.0; // TODO: Assign different values to different trajectory point

  // All actions start with up position first
  // TODO: Maybe start from a more accessible configuration?
  std::vector<double> config_up {0, -M_PI/2, 0, -M_PI/2, 0, 0}; // Up position
  trajectory_msgs::JointTrajectoryPoint joints_up;
  joints_up.positions = config_up;
  joints_up.velocities = zero_vector;
  joints_up.time_from_start = ros::Duration(2.0);
  trajectory.points.push_back(joints_up);
  ROS_INFO("Point 1:");
  debugPrintJoints(joints_up.positions);


  // Then drive to above position
  trajectory_msgs::JointTrajectoryPoint joints_above;
  joints_above.positions = config_above;
  joints_above.velocities = zero_vector;
  joints_above.time_from_start = ros::Duration(time_required);
  trajectory.points.push_back(joints_above);
  ROS_INFO("Point 2:");
  debugPrintJoints(joints_above.positions);

  /*
  // Then drive to side position
  trajectory_msgs::JointTrajectoryPoint joints_side;
  joints_side.positions = config_side;
  joints_side.velocities = zero_vector;
  joints_side.time_from_start = ros::Duration(time_required * 2);
  trajectory.points.push_back(joints_side);
  ROS_INFO("Point 3:");
  debugPrintJoints(joints_side.positions);
  */

  // Finally, drive to block position
  trajectory_msgs::JointTrajectoryPoint joints_block;
  joints_block.positions = config_block;
  joints_block.velocities = zero_vector;
  joints_block.time_from_start = ros::Duration(time_required * 2);
  trajectory.points.push_back(joints_block);
  ROS_INFO("Point 4:");
  debugPrintJoints(joints_block.positions);

  /* Send the trajectory */
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;

  ROS_INFO("Sending goal to action server");
  actionlib::SimpleClientGoalState status = action_client_->sendGoalAndWait(goal); // Will wait forever
  ROS_INFO("Action returned with status: %s", status.toString().c_str());

  return status;
}

/*******************************************************************
 *                        INVERSE KINEMATICS                       *
 *******************************************************************/

/**
 * Change from tf to row major array
 */
std::vector<double> TrajCtrl::transformToRowMajorTransform(tf::Transform transform)
{
  ROS_INFO("Changing from transform to row major array");

  tf::Matrix3x3 rotation = transform.getBasis();
  tf::Vector3 translation = transform.getOrigin();
  
  std::vector<double> rv{
      rotation[0].getX(), rotation[0].getY(), rotation[0].getZ(), translation.getX(), 
      rotation[1].getX(), rotation[1].getY(), rotation[1].getZ(), translation.getY(),
      rotation[2].getX(), rotation[2].getY(), rotation[2].getZ(), translation.getZ(),
                     0.0,                0.0,                0.0,                1.0};

  // DEBUG: print the array
  for (auto v: rv)
    std::cout << v << "," << std::flush;
  std::cout << std::endl;

  return rv;
}

/**
 * Cherry pick some suitable configurations for Jenga playing
 */
TrajCtrl::Configuration TrajCtrl::eliminateConfigurations(
    std::vector<TrajCtrl::Configuration> configurations)
{
  ROS_INFO("Eliminating configurations");

  std::vector<TrajCtrl::Configuration> rv;
  
  for(auto config: configurations)
  {
    ROS_INFO("This configuration:");
    debugPrintJoints(config);

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

    //if (config[WRIST_1_JOINT] > 0.4 || config[WRIST_1_JOINT] < 0.0)
    /* By observation:
     *   0.1  > wrist_1_joint > 0 // side 0, side 1
     *   3.14 > wrist_1_joint > 2.7 // side 2, side 3
     */
    if (config[SHOULDER_PAN_JOINT] > 0.0) // For side 0, side 1
    {
      if(config[WRIST_1_JOINT] < 0 || config[WRIST_1_JOINT] > 0.1)
            {
              ROS_INFO("--REJECTED: wrist_1_joint(PAN>0)");
              continue;
            }
    }
    else // For side 2, side 3
    {
      if(!(
          (config[WRIST_1_JOINT] < -2.7 && config[WRIST_1_JOINT] > -M_PI)||  // For side 2 
          (config[WRIST_1_JOINT] < 3 && config[WRIST_1_JOINT] > M_PI/2)// For side 3
          ) )
      {
        ROS_INFO("--REJECTED: wrist_1_joint(PAN<0)");
        continue;
      }
    }
/*
    if (!( // >>>NOT<<<
        (config[WRIST_1_JOINT] > 0    && config[WRIST_1_JOINT] <  0.1 && pan_flag) || // For side 0 and side 1
        (config[WRIST_1_JOINT] < -2.7 && config[WRIST_1_JOINT] > -M_PI && ~pan_flag)||  // For side 2 
        (config[WRIST_1_JOINT] < 3 && config[WRIST_1_JOINT] > M_PI/2 && ~pan_flag)// For side 3
        ) )
    {
      ROS_INFO("--REJECTED: wrist_1_joint");
      continue;
    }
    */

    // If reached here, this config is probably good
    ROS_INFO("--OK!");
    rv.push_back(config);
  }

  ROS_INFO("Found %lu probably good solutions", rv.size());
  if (!rv.size())
    ROS_WARN("NO GOOD SOLUTIONS FOUND!");

  // TODO: Pick one configuration if rv.size()>1
  return rv[0]; // for now: pick the first one
}

// Pick the configuration that requires least effort from start configuration
TrajCtrl::Configuration TrajCtrl::pickMinimumEffortConfiguration(
    std::vector<TrajCtrl::Configuration> possible_configurations,
    TrajCtrl::Configuration start_configuration)
{
  ROS_INFO("Picking minimum effort configuration; start config:");
  debugPrintJoints(start_configuration);

  //for (auto config: possible_configurations)
  //  debugPrintJoints(config);

  std::vector<double> weight { 1, 1, 1, 1, 1, 1};
  
  double min_difference = 99999.99;
  TrajCtrl::Configuration min_config;

  // Find the configuration that is minimal differnece from start_configuration
  for (auto config: possible_configurations)
  {
    double difference = 0;
    for (int i = 0; i < 6; ++i)
      difference += weight[i] * std::abs(config[i] - start_configuration[i]);

    if (min_difference > difference) // This config is better
    {
      min_difference = difference;
      min_config = config;
    }
  }

  ROS_INFO("Minimal effort: %f, configuration:", min_difference);
  debugPrintJoints(min_config);

  return min_config;
}

/**
 * Call inverse kinematics service to get possible configurations
 */
std::vector<TrajCtrl::Configuration> TrajCtrl::getInverseConfigurations(tf::Transform target_transform)
{
  /* Prepare data for inverse kinematics service */
  jenga_ur5_control::Ur5InverseKinematics srv;
  srv.request.T = transformToRowMajorTransform(target_transform);

  // DEBUG: publish this frame
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(target_transform, ros::Time::now(), "base_link", "target_transform") );

  /* Call inverse kinematics service */
  std::vector<TrajCtrl::Configuration> rv;

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
    debugBreak();
    return rv; // Empty vector
  }

  // DEBUG: print out the results
  ROS_INFO("Inverse kinematics results (%d solutions found):", srv.response.num_sols);
  //debugPrintJoints(srv.response.q_sols); // this should work
  
  /* Store the results in a more easily accessible vector format */
  for (int sol = 0; sol < srv.response.num_sols; ++sol)
  {
    TrajCtrl::Configuration this_config;
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

// Check and update game state. Return first empty block slot
int TrajCtrl::checkGameState()
{
  int block = -2;
  for (int i = 0; i < 3; ++i)
  {
    if (!top_status_[i]) {
      // Found empty spot
      top_status_[i] = 1; // Fill this spot
      block = i - 1; // Get block number (-1, 0, or 1)
      break; // Place in sequence -1 -> 0 -> 1
    }
  }

  if (block == -2){
    // No empty slot found
    current_level_ += 1; // Increment level
    top_orientation_ = !top_orientation_; // Change orientation
    std::fill( top_status_.begin(), top_status_.end(), 0 ); // Reset top status

    block = -1; // Set block number
  }

  return block;
}

/*******************************************************************
 *                         TF MANIPULATION                         *
 *******************************************************************/
/** 
 * Get a ee_link transformation for the specified block, depending on input action
 */
tf::Transform TrajCtrl::getTransformFor(int mode, int side, int level, int block)
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
 * Wait and return the transfrom by the input name
 */
tf::Transform TrajCtrl::retrieveTransform(std::string frame_name)
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
 *Get the corresponding tf transform from target block 
 */
tf::Transform TrajCtrl::targetBlockToTargetTransform(int side, int level, int block)
{
  /* Look up the transform to the side of the tower */
  std::string frame_name_side("roadmap_side" + std::to_string(side) );
  tf::Transform tf_side = retrieveTransform(frame_name_side);
  
  /* Retrieve from parameter server the level at which the side waypoint is located */
  int side_level;
  nh_.param("side_level", side_level, 9); // Default value = 9
  
  /* Calculate relative level and block offset */
  // Jenga block dimensions: 15x25x75 mm
  double y_offset = 0.015 * (side < 2? (side_level - level): (level - side_level));
  double x_offset = 0.025 * block;

  ROS_INFO("x_offset = %f, y_offset = %f", x_offset, y_offset);

  /* Create a new tf for this block based on the side waypoint */
  tf::Transform tf_block(tf::Quaternion::getIdentity(), tf::Vector3(x_offset, y_offset, 0.0));

  /* Publish this transformation (NOTE: this is relative to side waypoint) */
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(tf_block, ros::Time::now(), frame_name_side, "tf_block"));

  /* Get the block transform >>>from base_link<<< */
  tf_block = retrieveTransform("tf_block");

  /* Show this transform on rviz */
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(tf_block, ros::Time::now(), "base_link", "tf_block_ee_link") );
      
  return tf_block;  
}

/**
 * Compensate ee_link -> tool0 -> tool_{gripper/range_finder/probe} 
 */
tf::Transform TrajCtrl::compensateEELinkToTool0(tf::Transform transform)
{
  tf::Transform ee_link_to_tool0( tf::Quaternion(-0.5, 0.5, -0.5, 0.5) ); 
  transform = transform * ee_link_to_tool0.inverse();

  return transform;
}
tf::Transform TrajCtrl::compensateEELinkToGripper(tf::Transform transform)
{
  tf::Transform ee_link_to_tool0( tf::Quaternion(-0.5, 0.5, -0.5, 0.5) ); 
  tf::Transform tool0_to_tool_gripper( tf::Quaternion::getIdentity(), tf::Vector3(0, 0, 0.072646) );

  tf::Transform ee_link_to_tool_gripper = ee_link_to_tool0 * tool0_to_tool_gripper;

  transform = transform * ee_link_to_tool_gripper.inverse();

  return transform;
}
tf::Transform TrajCtrl::compensateEELinkToRangeFinder(tf::Transform transform)
{
  tf::Transform ee_link_to_tool0( tf::Quaternion(-0.5, 0.5, -0.5, 0.5) ); 
  tf::Transform tool0_to_tool_range_finder( tf::Quaternion::getIdentity(), tf::Vector3(0, -0.02942, 0.059234) );

  tf::Transform ee_link_to_tool_range_finder = ee_link_to_tool0 * tool0_to_tool_range_finder;

  transform = transform * ee_link_to_tool_range_finder.inverse();

  return transform;
}
tf::Transform TrajCtrl::compensateEELinkToProbe(tf::Transform transform)
{
  tf::Transform ee_link_to_tool0( tf::Quaternion(-0.5, 0.5, -0.5, 0.5) ); 
  tf::Transform tool0_to_tool_probe( tf::Quaternion(0.0, M_PI/4, 0.0, M_PI/4), tf::Vector3(0.0795, 0, 0.026146) );

  tf::Transform ee_link_to_tool_probe = ee_link_to_tool0 * tool0_to_tool_probe;

  transform = transform * ee_link_to_tool_probe.inverse();

  return transform;
}

/*******************************************************************
 *                      TRAJECTORY GENERATION                      *
 *******************************************************************/

/**
 * Send the trajectory to action server and block until execution returned 
 */
actionlib::SimpleClientGoalState TrajCtrl::executeTrajectoryGoal(
    control_msgs::FollowJointTrajectoryGoal trajectory_goal)
{
  ROS_INFO("Sending goal to action server");

  actionlib::SimpleClientGoalState status = action_client_->sendGoalAndWait(trajectory_goal); // Will wait forever

  return status;
}

/**
 * Generate a trajectory goal object that drive the arm from current position to input target_transform.
 */
control_msgs::FollowJointTrajectoryGoal TrajCtrl::generateTrajectory(int side, tf::Transform tf_target)
{
  ROS_INFO("Generating trajectory to side%d", side);

  /* Lookup transform on the route */
  std::string frame_name_above("roadmap_above" + std::to_string(side) );
  std::string frame_name_side("roadmap_side" + std::to_string(side) );

  tf::Transform tf_above, tf_side;
  tf_above = compensateEELinkToTool0( retrieveTransform(frame_name_above) );
  tf_side = compensateEELinkToTool0( retrieveTransform(frame_name_side) );

  // DEBUG: show the frames
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(tf_above, ros::Time::now(), "base_link", "tf_above_ee_link"));
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(tf_side, ros::Time::now(), "base_link", "tf_side_ee_link"));

  /* Call inverse kinematics to get possible configurations */
  // TODO: Pre-calculate the waypoint configurations?
  std::vector<TrajCtrl::Configuration> inv_configs_above, inv_configs_side, inv_configs_target;
  inv_configs_above = getInverseConfigurations(tf_above);
  inv_configs_target = getInverseConfigurations(tf_target);

  /* Eliminate configurations to only one per waypoint */
  TrajCtrl::Configuration config_above, config_target;
  // Eliminate configurations for roadmap_above using heuristic method
  config_above = eliminateConfigurations(inv_configs_above);

  // Eliminate configurations for roadmap_side and roadmap_block by least difference to its predecessor
  // The robot moves in this order: roadmap_above -> roadmap_side -> roadmap_block
  //config_side = pickMinimumEffortConfiguration(inv_configs_side, config_above);
  config_target = pickMinimumEffortConfiguration(inv_configs_target, config_above);

  /* Drive the arm to the waypoints sequentially */
  ROS_INFO("[generateTrajectory] Initializing new trajectory");
  trajectory_msgs::JointTrajectory trajectory; // Initialize new trajectory
  trajectory.joint_names = UR_JOINT_NAMES_;
  std::vector<double> zero_vector{0, 0, 0, 0, 0, 0};
  //double time_required = 4.0; // TODO: Assign different values to different trajectory point
/*
  // All actions start with up position first
  // TODO: Maybe start from a more accessible configuration?
  std::vector<double> config_up {0, -M_PI/2, 0, -M_PI/2, 0, 0}; // Up position
  trajectory_msgs::JointTrajectoryPoint joints_up;
  joints_up.positions = config_up;
  joints_up.velocities = zero_vector;
  joints_up.time_from_start = ros::Duration(2.0);
  trajectory.points.push_back(joints_up);
  ROS_INFO("Point 1:");
  debugPrintJoints(joints_up.positions);
*/
  // All actions start with current position
  trajectory_msgs::JointTrajectoryPoint joints_current;
  joints_current.positions = getCurrentJointState().position;
  joints_current.velocities = zero_vector;
  joints_current.time_from_start = ros::Duration(0.0); // Start immediately
  trajectory.points.push_back(joints_current);

  ROS_INFO("[generateTrajectory] Point 1:");
  debugPrintJoints(joints_current.positions);

  // Then drive to above position
  trajectory_msgs::JointTrajectoryPoint joints_above;
  joints_above.positions = config_above;
  joints_above.velocities = zero_vector;
  joints_above.time_from_start = ros::Duration(3.0); // TODO: tune this time
  trajectory.points.push_back(joints_above);
  ROS_INFO("[generateTrajectory] Point 2:");
  debugPrintJoints(joints_above.positions);

  /*
  // Then drive to side position
  trajectory_msgs::JointTrajectoryPoint joints_side;
  joints_side.positions = config_side;
  joints_side.velocities = zero_vector;
  joints_side.time_from_start = ros::Duration(time_required * 2);
  trajectory.points.push_back(joints_side);
  ROS_INFO("Point 3:");
  debugPrintJoints(joints_side.positions);
  */

  // Finally, drive to target
  trajectory_msgs::JointTrajectoryPoint joints_target;
  joints_target.positions = config_target;
  joints_target.velocities = zero_vector;
  joints_target.time_from_start = ros::Duration(6.0); // TODO: tune this time
  trajectory.points.push_back(joints_target);
  ROS_INFO("[generateTrajectory] Point 3:");
  debugPrintJoints(joints_target.positions);

  /* Build the goal message */
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;

  return goal;
}

control_msgs::FollowJointTrajectoryGoal TrajCtrl::generateHomingTrajectory(int side)
{
  ROS_INFO("Generating trajectory back to home position from side%d", side);

  bool return_direct = false;
  if(side == 5){
    return_direct = true;
    side = 0; // dummy number to go through most of the useless code in this situation
  }

  /* Lookup transform on the route */
  std::string frame_name_above("roadmap_above" + std::to_string(side) );
  std::string frame_name_side("roadmap_side" + std::to_string(side) );

  tf::Transform tf_above, tf_side;
  tf_above = compensateEELinkToTool0( retrieveTransform(frame_name_above) );
  tf_side = compensateEELinkToTool0( retrieveTransform(frame_name_side) );

  // DEBUG: show the frames
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(tf_above, ros::Time::now(), "base_link", "tf_above_ee_link"));
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(tf_side, ros::Time::now(), "base_link", "tf_side_ee_link"));

  /* Get the inverse transform */
  std::vector<Configuration> inv_configs_above = getInverseConfigurations(tf_above);

  /* Eliminate to only one configuration */
  TrajCtrl::Configuration config_above = eliminateConfigurations(inv_configs_above);

  /* Drive the arm to the waypoints sequentially */
  ROS_INFO("[generateHomingTrajectory] Initializing new trajectory");
  trajectory_msgs::JointTrajectory trajectory; // Initialize new trajectory
  trajectory.joint_names = UR_JOINT_NAMES_;
  std::vector<double> zero_vector{0, 0, 0, 0, 0, 0};
  //double time_required = 4.0; // TODO: Assign different values to different trajectory point

  // All actions start with current position
  trajectory_msgs::JointTrajectoryPoint joints_current;
  joints_current.positions = getCurrentJointState().position;
  joints_current.velocities = zero_vector;
  joints_current.time_from_start = ros::Duration(0.0); // Start immediately
  trajectory.points.push_back(joints_current);
  ROS_INFO("[generateHomingTrajectory] Point 1:");
  debugPrintJoints(joints_current.positions);

  if (!return_direct) // Not returning directly, i.e. robot is currently in one of four sides
  {
      // Then drive to above position
    trajectory_msgs::JointTrajectoryPoint joints_above;
    joints_above.positions = config_above;
    joints_above.velocities = zero_vector;
    joints_above.time_from_start = ros::Duration(2.0); // TODO: tune this time
    trajectory.points.push_back(joints_above);
    ROS_INFO("[generateHomingTrajectory] Point 2:");
    debugPrintJoints(joints_above.positions);
  }

  // Finally, drive to home position
  trajectory_msgs::JointTrajectoryPoint joints_up;
  joints_up.positions = HOME_CONFIG_;
  joints_up.velocities = zero_vector;
  joints_up.time_from_start = ros::Duration( (side < 5)? 5.0 : 3.0 );
  trajectory.points.push_back(joints_up);
  ROS_INFO("[generateHomingTrajectory] Point 3:");
  debugPrintJoints(joints_up.positions);

  /* Build the goal message */
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;

  return goal;
}

/*******************************************************************
 *                     JENGA PLAYING FUNCTIONS                     *
 *******************************************************************/
/**
 * Move in and push 25mm
 */
actionlib::SimpleClientGoalState TrajCtrl::executeProbingAction()
{
  ROS_INFO("Execting probing action");

  /* Get the probe transform */
  tf::Transform tf_probe = retrieveTransform("tool_probe");

  /* Calculate target transform */
  // Move in 75mm (0.075m) in z direction to touch the block, then try push 25mm (0.025m)
  // ===> Move 100mm (0.1m) in total
  //tf::Vector3 target_translation = tf_probe.getOrigin();
  //target_translation.setZ( target_translation.getZ() + 0.1 );
  tf::Transform tf_target( tf::Quaternion::getIdentity(), tf::Vector3(0, 0, 0.1) );

  tf_target = compensateEELinkToProbe(tf_probe * tf_target);

  /* Get the configuration for target transform */
  TrajCtrl::Configuration config_start = getCurrentJointState().position; // Save current configuration for when we need to abort
  TrajCtrl::Configuration config_target = 
      pickMinimumEffortConfiguration( getInverseConfigurations(tf_target), config_start );

  /* Initialize trajectory */
  ROS_INFO("[executeProbingAction] Initializing trajectory");
  trajectory_msgs::JointTrajectory trajectory; // Initialize new trajectory
  trajectory.joint_names = UR_JOINT_NAMES_;
  std::vector<double> zero_vector{0, 0, 0, 0, 0, 0};

  // All actions start with current position
  trajectory_msgs::JointTrajectoryPoint joints_current;
  joints_current.positions = getCurrentJointState().position;
  joints_current.velocities = zero_vector;
  joints_current.time_from_start = ros::Duration(0.0); // Start immediately
  trajectory.points.push_back(joints_current);
  ROS_INFO("[executeProbingAction] Point 1:");
  debugPrintJoints(joints_current.positions);

  // Then drive to target position
  trajectory_msgs::JointTrajectoryPoint joints_target;
  joints_target.positions = config_target;
  joints_target.positions[5] = joints_current.positions[5]; // Ensure end effector does not rotate 360 degrees
  joints_target.velocities = zero_vector;
  joints_target.time_from_start = ros::Duration(5.0); // 100mm in 5 sec = 2cm/sec
  trajectory.points.push_back(joints_target);
  ROS_INFO("[executeProbingAction] Point 2:");
  debugPrintJoints(joints_target.positions);

  /* Send the trajectory */
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  executeTrajectoryGoal(goal); 
  // It actually does not matter if probing is successful or not. We always want the probe to return to starting config
  // The subroutine that signals success or fail is responsible of letting others know the result.

  /* Move back to starting position */
  // Clear the trajectory
  trajectory.points = std::vector<trajectory_msgs::JointTrajectoryPoint>();

  // Start with current position
  joints_current.positions = getCurrentJointState().position;
  trajectory.points.push_back(joints_current);
  ROS_INFO("[executeProbingAction] Return, Point 1:");
  debugPrintJoints(joints_current.positions);

  // Then drive back to start position
  joints_target.positions = config_start;
  joints_target.time_from_start = ros::Duration(3.0);
  trajectory.points.push_back(joints_target);
  ROS_INFO("[executeProbingAction] Return, Point 2:");
  debugPrintJoints(joints_target.positions);

  /* Send the trajectory */
  goal.trajectory = trajectory;
  actionlib::SimpleClientGoalState status = executeTrajectoryGoal(goal);

  ROS_INFO("Probing action done, with status %s", status.toString().c_str() );
  return status;
}

/**
 * Move in and close gripper. Pull out and return to home configuration safely.
 */
actionlib::SimpleClientGoalState TrajCtrl::executeGrippingAction(int mode)
{
  ROS_INFO("Execting gripping action, close position: %s", (mode == GRIPPER_CLOSE_NARROW)? "SHORT" : "LONG");

  /* Get the gripper transform */
  tf::Transform tf_gripper = retrieveTransform("tool_gripper");

  /* Calculate target transformation */
  tf::Transform tf_target;
  if (mode == GRIPPER_CLOSE_NARROW)
  {
    // Originally 75mm away from the block, but the block is pushed out 25mm
    // ===> Move in 50mm + 5mm pull out safty margin = 55mm
    //tf::Vector3 target_translation = tf_gripper.getOrigin();
    //target_translation.setZ( target_translation.getZ() + 0.055 );
    tf_target = tf::Transform( tf::Quaternion::getIdentity(), tf::Vector3(0, 0, 0.055) );
  }
  else // close long
  {
    // Calculate the offset
    tf::Transform tf_block_above = retrieveTransform("roadmap_block_above");
    tf::Transform tf_block_place = retrieveTransform("roadmap_block_place");

    double above_offset = tf_block_above.getOrigin().getZ() - tf_block_place.getOrigin().getZ();
    tf_target = tf::Transform( tf::Quaternion::getIdentity(), tf::Vector3(0, 0, above_offset) );
  }

  tf_target = compensateEELinkToGripper(tf_gripper * tf_target);

  /* Get the configuration for target transform */
  TrajCtrl::Configuration config_start = getCurrentJointState().position; // Save current configuration for pulling out

  TrajCtrl::Configuration config_target = 
      pickMinimumEffortConfiguration( getInverseConfigurations(tf_target), config_start );

  /* Initialize trajectory */
  ROS_INFO("[executeGrippingAction] Initializing trajectory");
  trajectory_msgs::JointTrajectory trajectory; // Initialize new trajectory
  trajectory.joint_names = UR_JOINT_NAMES_;
  std::vector<double> zero_vector{0, 0, 0, 0, 0, 0};

  // All actions start with current position
  trajectory_msgs::JointTrajectoryPoint joints_current;
  joints_current.positions = getCurrentJointState().position;
  joints_current.velocities = zero_vector;
  joints_current.time_from_start = ros::Duration(0.0); // Start immediately
  trajectory.points.push_back(joints_current);
  ROS_INFO("[executeGrippingAction] Point 1:");
  debugPrintJoints(joints_current.positions);

  // Then drive to target position
  trajectory_msgs::JointTrajectoryPoint joints_target;
  joints_target.positions = config_target;
  joints_target.velocities = zero_vector;
  joints_target.time_from_start = ros::Duration(2.0);
  trajectory.points.push_back(joints_target);
  ROS_INFO("[executeGrippingAction] Point 2:");
  debugPrintJoints(joints_target.positions);

  /* Send the trajectory */
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  executeTrajectoryGoal(goal); 

  /* Close gripper */
  // Prepare and send the command message
  publishToolCommand(mode);
  
  // Wait until the gripper is closed
  blockUntilToolFeedback(jenga_msgs::EndEffectorFeedback::ACK_GRIPPER_CLOSED);

  /* Move back to starting position */
  // Clear the trajectory
  trajectory.points = std::vector<trajectory_msgs::JointTrajectoryPoint>();

  // Start with current position
  joints_current.positions = getCurrentJointState().position;
  trajectory.points.push_back(joints_current);
  ROS_INFO("[executeGrippingAction] Return, Point 1:");
  debugPrintJoints(joints_current.positions);

  // Then drive back to start position
  joints_target.positions = config_start;
  joints_target.time_from_start = ros::Duration(3.0);
  trajectory.points.push_back(joints_target);
  ROS_INFO("[executeGrippingAction] Return, Point 2:");
  debugPrintJoints(joints_target.positions);

  /* Send the trajectory */
  goal.trajectory = trajectory;
  actionlib::SimpleClientGoalState status = executeTrajectoryGoal(goal);

  ROS_INFO("Gripping action done, with status %s", status.toString().c_str() );
  return status;
}

/**
 * Move to the +x direction 25mm, then to -x direction 50mm, then return to start position
 */
actionlib::SimpleClientGoalState TrajCtrl::executeRangeFindingAction()
{
  ROS_INFO("Executing range finding action");

  /* Get the range finder transform */
  tf::Transform tf_range_finder = retrieveTransform("tool_range_finder");

  /* Calculate target transformations */
  // Move to the +x direction 25mm, then to -x direction 50mm, then return to start position
  //tf::Vector3 target_translation = tf_range_finder.getOrigin();
  //target_translation.setZ( target_translation.getZ() + 0.025 );
  //tf::Transform tf_left(tf_range_finder.getRotation(), target_translation);
  //target_translation.setZ( target_translation.getZ() - 0.050 );
  //tf::Transform tf_right(tf_range_finder.getRotation(), target_translation);

  tf::Transform tf_left( tf::Quaternion::getIdentity(), tf::Vector3(0.025, 0, 0) );
  tf::Transform tf_right( tf::Quaternion::getIdentity(), tf::Vector3(-0.025, 0, 0) );

  tf_left = compensateEELinkToRangeFinder(tf_range_finder * tf_left);
  tf_right = compensateEELinkToRangeFinder(tf_range_finder * tf_right);

  /* Get the configuration for target transforms */
  TrajCtrl::Configuration config_start = getCurrentJointState().position; // Save current configuration for pulling out
  TrajCtrl::Configuration config_left = 
      pickMinimumEffortConfiguration( getInverseConfigurations(tf_left), config_start );
  TrajCtrl::Configuration config_right = 
      pickMinimumEffortConfiguration( getInverseConfigurations(tf_right), config_start );

  /* Move start -> left -> right -> start */

  /* Initialize trajectory */
  ROS_INFO("[executeRangeFindingAction] Initializing trajectory");
  trajectory_msgs::JointTrajectory trajectory; // Initialize new trajectory
  trajectory.joint_names = UR_JOINT_NAMES_;
  std::vector<double> zero_vector{0, 0, 0, 0, 0, 0};
  double time_required = 2.0;

  ROS_INFO("configurations:");
  debugPrintJoints(config_start);
  debugPrintJoints(config_left);
  debugPrintJoints(config_right);

  // All actions start with current position
  trajectory_msgs::JointTrajectoryPoint joints_start;
  joints_start.positions = getCurrentJointState().position;
  joints_start.velocities = zero_vector;
  joints_start.time_from_start = ros::Duration(0.0); // Start immediately
  trajectory.points.push_back(joints_start);

  // Then drive to left position
  trajectory_msgs::JointTrajectoryPoint joints_left;
  joints_left.positions = config_left;
  joints_left.velocities = zero_vector;
  joints_left.time_from_start = ros::Duration(time_required * 1);
  trajectory.points.push_back(joints_left);

  // Then drive to right position
  trajectory_msgs::JointTrajectoryPoint joints_right;
  joints_right.positions = config_right;
  joints_right.velocities = zero_vector;
  joints_right.time_from_start = ros::Duration(time_required * 3);
  trajectory.points.push_back(joints_right);

  // Drive back to start position again
  joints_start.time_from_start = ros::Duration(time_required * 4);
  trajectory.points.push_back(joints_start);

  // DEBUG
  ROS_INFO("[executeRangeFindingAction] trajectory size: %lu", trajectory.points.size());
  for (int i = 0; i < trajectory.points.size(); ++i)
  {
    ROS_INFO("[executeRangeFindingAction] Point%d:", i);
    debugPrintJoints(trajectory.points[i].positions);
  }

  /* Signal start collecting data */
  // TODO

  /* Send the trajectory */
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  actionlib::SimpleClientGoalState status = executeTrajectoryGoal(goal);

  ROS_INFO("Range finding action done, with status %s", status.toString().c_str() );
  return status;
}
/* 1. Move from current position (assumed at home) to drop location
 * 2. Open gripper to drop the block
 * 3. Move to roadmap_block_above
 * 4. Execute gripping action, close grip on long side
 */
actionlib::SimpleClientGoalState TrajCtrl::executeGripChangeAction()
{
  ROS_INFO("Executing grip change action.");
  checkRobotInHomeConfig();

  /* Retrieve necessary transformations and calculate necessary parameters */
  tf::Transform tf_block_drop = retrieveTransform("roadmap_block_drop");
  tf::Transform tf_block_above = retrieveTransform("roadmap_block_above");

  tf_block_drop = compensateEELinkToGripper(tf_block_drop);
  tf_block_above = compensateEELinkToGripper(tf_block_above);

  tf_broadcaster_.sendTransform(
      tf::StampedTransform(tf_block_drop, ros::Time::now(), "base_link", "tf_block_drop_ee_link"));

  /* Move from current position (assumed at home) to drop location */
  // Get a suitable configuration for target transform
  TrajCtrl::Configuration config_approximate {2.12820, -0.92279, 1.49792, -1.17135, -1.73198, -3.14159};
  TrajCtrl::Configuration config_above = pickMinimumEffortConfiguration( getInverseConfigurations(tf_block_above), config_approximate );
  //TrajCtrl::Configuration config_above = eliminateConfigurations( getInverseConfigurations(tf_block_above) );
  TrajCtrl::Configuration config_drop = pickMinimumEffortConfiguration( getInverseConfigurations(tf_block_drop), config_above );

  // Initialize trajectory
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names = UR_JOINT_NAMES_;
  std::vector<double> zero_vector {0, 0, 0, 0, 0, 0};

  // Start with current position
  trajectory_msgs::JointTrajectoryPoint joints_current;
  joints_current.positions = getCurrentJointState().position;
  joints_current.velocities = zero_vector;
  joints_current.time_from_start = ros::Duration(0.0); // Start immediately
  trajectory.points.push_back(joints_current);

  // Move pan and wrist 3 joints first
  trajectory_msgs::JointTrajectoryPoint joints_intermediate;
  joints_intermediate.positions = joints_current.positions;
  joints_intermediate.positions[0] = config_drop[0];
  joints_intermediate.positions[5] = config_drop[5];
  joints_intermediate.velocities = zero_vector;
  joints_intermediate.time_from_start = ros::Duration(2.0);
  trajectory.points.push_back(joints_intermediate);

  // Move to drop location
  trajectory_msgs::JointTrajectoryPoint joints_drop;
  joints_drop.positions = config_drop;
  joints_drop.velocities = zero_vector;
  joints_drop.time_from_start = ros::Duration(5.0);
  trajectory.points.push_back(joints_drop);

  // Send the trajectory
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  actionlib::SimpleClientGoalState status = executeTrajectoryGoal(goal);
  ROS_INFO("Moving to drop position done, with status %s", status.toString().c_str() );

  debugBreak();

  /* Open gripper to drop the block */
  // TODO
  ROS_INFO("OPENING GRIPPER");

  /* Move to the position above the dropped block */
  // Reset the trajectory points
  trajectory.points = std::vector<trajectory_msgs::JointTrajectoryPoint>();

  // Start with current position
  joints_current.positions = getCurrentJointState().position;
  trajectory.points.push_back(joints_current);

  // Move to the position above the block
  trajectory_msgs::JointTrajectoryPoint joints_above;
  joints_above.positions = config_above;
  joints_above.velocities = zero_vector;
  joints_above.time_from_start = ros::Duration(2.0);
  trajectory.points.push_back(joints_above);

  // Send the trajectory
  goal.trajectory = trajectory; 
  status = executeTrajectoryGoal(goal);
  ROS_INFO("Moving to position above the block done, with status %s", status.toString().c_str() );

  debugBreak();

  /* Pick up the block on the long side */
  status = executeGrippingAction(GRIPPER_CLOSE_WIDE);
  ROS_INFO("Gripping the block done, with status %s", status.toString().c_str() );

  return status;
}

actionlib::SimpleClientGoalState TrajCtrl::executePlaceBlockAction()
{
  ROS_INFO("Executing place block action.");

  /* Get the gripper transform */
  tf::Transform tf_gripper = retrieveTransform("tool_gripper");

  /* Retrieve from parameter server the level at which the above waypoint is located */
  int direct_above_level;
  nh_.param("direct_above_level", direct_above_level, 30); // Default value = 30

  /* Calculate target transformation */
  // Originally ${direct_above_level} levels above. Tower is currently ${current_level_} levels.
  // Need to move down ${direct_above_level} - ${current_level_} levels
  double z_offset = 0.015 * (direct_above_level - current_level_);
  tf::Transform tf_target(tf::Quaternion::getIdentity(), tf::Vector3(0, 0, z_offset));

  tf_target = compensateEELinkToGripper(tf_gripper * tf_target);

  /* Get the configuration for target transform */
  TrajCtrl::Configuration config_start = getCurrentJointState().position; // Save current configuration for when action is done
  TrajCtrl::Configuration config_target = 
      pickMinimumEffortConfiguration( getInverseConfigurations(tf_target), config_start );

  /* Initialize trajectory */
  ROS_INFO("[executePlaceBlockAction] Initializing trajectory");
  trajectory_msgs::JointTrajectory trajectory; // Initialize new trajectory
  trajectory.joint_names = UR_JOINT_NAMES_;
  std::vector<double> zero_vector{0, 0, 0, 0, 0, 0};

  // All actions start with current position
  trajectory_msgs::JointTrajectoryPoint joints_current;
  joints_current.positions = getCurrentJointState().position;
  joints_current.velocities = zero_vector;
  joints_current.time_from_start = ros::Duration(0.0); // Start immediately
  trajectory.points.push_back(joints_current);
  ROS_INFO("[executePlaceBlockAction] Point 1:");
  debugPrintJoints(joints_current.positions);

  // Then drive to target position
  trajectory_msgs::JointTrajectoryPoint joints_target;
  joints_target.positions = config_target;
  joints_target.velocities = zero_vector;
  joints_target.time_from_start = ros::Duration(3.0);
  trajectory.points.push_back(joints_target);
  ROS_INFO("[executePlaceBlockAction] Point 2:");
  debugPrintJoints(joints_target.positions);

  /* Send the trajectory */
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  executeTrajectoryGoal(goal); 

  /* Open gripper */
  // TODO

  /* Move back to starting position */
  // Clear the trajectory
  trajectory.points = std::vector<trajectory_msgs::JointTrajectoryPoint>();

  // Start with current position
  joints_current.positions = getCurrentJointState().position;
  trajectory.points.push_back(joints_current);
  ROS_INFO("[executePlaceBlockAction] Return, Point 1:");
  debugPrintJoints(joints_current.positions);

  // Then drive back to start position
  joints_target.positions = config_start;
  joints_target.time_from_start = ros::Duration(3.0);
  trajectory.points.push_back(joints_target);
  ROS_INFO("[executePlaceBlockAction] Return, Point 2:");
  debugPrintJoints(joints_target.positions);

  /* Send the trajectory */
  goal.trajectory = trajectory;
  actionlib::SimpleClientGoalState status = executeTrajectoryGoal(goal);

  ROS_INFO("Placing action done, with status %s", status.toString().c_str() );
  return status;
}
actionlib::SimpleClientGoalState TrajCtrl::moveToActionPosition(int action, int side, int level, int block)
{
  if (action < 0 || action > 3)
  {
    ROS_ERROR("Unknown action number %d", action);
    return actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::StateEnum::LOST);
  }

  checkRobotInHomeConfig(); // Issue a warning if robot is not currently in home config

  /* Get target transform at the specified block location for specified action */
  tf::Transform tf_target = getTransformFor(action, side, level, block);

  /* Generate and execute the trajectory to this transformation */
  control_msgs::FollowJointTrajectoryGoal trajectory_goal = generateTrajectory(side, tf_target); 
  actionlib::SimpleClientGoalState status = executeTrajectoryGoal(trajectory_goal);
  
  ROS_INFO("Moving action done, with status %s", status.toString().c_str() );
  return status;
}

actionlib::SimpleClientGoalState TrajCtrl::moveToHomePosition(int side)
{
  /* Generate and execute the trajectory back to home position */
  control_msgs::FollowJointTrajectoryGoal trajectory_goal = generateHomingTrajectory(side);
  actionlib::SimpleClientGoalState status = executeTrajectoryGoal(trajectory_goal);
  
  ROS_INFO("Return to home position done, with status %s", status.toString().c_str() );
  return status;
}
/*
actionlib::SimpleClientGoalState TrajCtrl::moveToGripChangePosition()
{
  // TODO
  return actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::StateEnum::LOST);
}
*/
/**
 * Move to a location above an empty block slot
 */
actionlib::SimpleClientGoalState TrajCtrl::moveToPlaceBlockPosition()
{
  ROS_INFO("Moving to place block position");

  tf::Transform tf_direct_above = retrieveTransform("roadmap_direct_above"); // Get the transform to top of the tower

  /* Calculate target transform based on current game state */
  // true: x direction; false: rotate z 90 deg, then x direction
  tf::Transform tf_target;
  int block = checkGameState(); // Get empty block slot
  if (top_orientation_)
    tf_target = tf::Transform( tf::Quaternion::getIdentity(), tf::Vector3(0, block * 0.015, 0) );
  else
  {
    tf::Quaternion q; q.setRPY(0, 0, M_PI/2); // Rotate +z 90 degrees
    tf_target = tf::Transform( q, tf::Vector3(block * 0.015, 0, 0) );
  }

  tf_target = compensateEELinkToGripper(tf_direct_above * tf_target);

  ROS_INFO("current_level_: %d, block: %d", current_level_, block);

  /* Get the configuration for target transform */
  TrajCtrl::Configuration config_start = getCurrentJointState().position; // Save current configuration for when action is done
  TrajCtrl::Configuration config_target = 
      pickMinimumEffortConfiguration( getInverseConfigurations(tf_target), config_start );

  /* Initialize trajectory */
  ROS_INFO("[executePlaceBlockAction] Initializing trajectory");
  trajectory_msgs::JointTrajectory trajectory; // Initialize new trajectory
  trajectory.joint_names = UR_JOINT_NAMES_;
  std::vector<double> zero_vector{0, 0, 0, 0, 0, 0};

  // All actions start with current position
  trajectory_msgs::JointTrajectoryPoint joints_current;
  joints_current.positions = getCurrentJointState().position;
  joints_current.velocities = zero_vector;
  joints_current.time_from_start = ros::Duration(0.0); // Start immediately
  trajectory.points.push_back(joints_current);
  ROS_INFO("[executePlaceBlockAction] Point 1:");
  debugPrintJoints(joints_current.positions);

  // Then drive to target position
  trajectory_msgs::JointTrajectoryPoint joints_target;
  joints_target.positions = config_target;
  joints_target.velocities = zero_vector;
  joints_target.time_from_start = ros::Duration(2.0);
  trajectory.points.push_back(joints_target);
  ROS_INFO("[executePlaceBlockAction] Point 2:");
  debugPrintJoints(joints_target.positions);

  /* Send the trajectory */
  control_msgs::FollowJointTrajectoryGoal goal; 
  goal.trajectory = trajectory;
  actionlib::SimpleClientGoalState status = executeTrajectoryGoal(goal);
  
  ROS_INFO("Moving to place block poistion done, with status %s", status.toString().c_str() );
  return status;
} 


/*******************************************************************
 *                    QUALITY OF LIFE DEBUGGING                    *
 *******************************************************************/
/**
 * Check the current configuration of the robot to see if it matches home configuration
 * Issues a warning if it is not in home configuration.
 */
void TrajCtrl::checkRobotInHomeConfig()
{
  ROS_INFO("Checking robot state");
  double delta = 0.001;
  bool flag = false;

  ros::spinOnce();

  for (int i = 0; i < 9; ++i)
  {
    double q = joint_state_.position[i];
    std::cout << q << std::endl;
    if ( (q > HOME_CONFIG_[i] + delta) || (q < HOME_CONFIG_[i] - delta) )
    {
      flag = true;
      ROS_WARN("Robot is currently not in home configuration when a moving action is called");
      break;
    }
  }
}

void TrajCtrl::publishToolCommand(int command_code)
{
  ROS_INFO("Publishing command %d", command_code);
  jenga_msgs::EndEffectorControl command_msg;
  command_msg.header.stamp = ros::Time::now();
  command_msg.command_code = (uint8_t) command_code;

  tool_command_publisher_.publish(command_msg);
}
/**
 * Block until a feedback from the tool is received
 */
bool TrajCtrl::blockUntilToolFeedback(int expected_feedback_code)
{
  ROS_INFO("Waiting for feedback %d from the tool...", expected_feedback_code);
  // Spin until flag is raised. Will wait forever
  // Flag is raised when feedbackCallback is called by the subscriber
  while (!tool_feedback_flag_)
  {
    ros::spinOnce(); // Process callbacks
    ROS_INFO_DELAYED_THROTTLE(30, "Still waiting...");
  }

  // Read feedback code and reset flag
  int received_feedback_code = tool_feedback_code_;
  tool_feedback_flag_ = false;

  ROS_INFO("Received %d", received_feedback_code);

  return (received_feedback_code == expected_feedback_code);
}
void TrajCtrl::feedbackCallback(const jenga_msgs::EndEffectorFeedback::ConstPtr& msg)
{
  if (tool_feedback_flag_)
    ROS_WARN("Feedback received but flag is true! Overwriting feedback code.");
  
  tool_feedback_code_ = msg->feedback_code;
  tool_feedback_flag_ = true; // Raise flag to signal the receptance of a feedback from tool
}
void TrajCtrl::probeCallback(const jenga_msgs::Probe::ConstPtr& msg)
{
  float force = msg->data;
  // TODO
}
void TrajCtrl::rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
  // TODO
}

void TrajCtrl::debugPrintJoints(TrajCtrl::Configuration joints)
{
  for (auto q : joints) 
    ROS_DEBUG_STREAM(q << " " << std::flush);
  ROS_DEBUG_STREAM(std::endl);
}

void TrajCtrl::debugBreak()
{
  char c;
  std::cin >> c;
  ros::spinOnce();
  //ros::Duration(3.0).sleep();
}

void TrajCtrl::debugTestFlow()
{
  int side = 0;
  int level = 9;
  int block = 0;

    /* Move arm to probing position */
  actionlib::SimpleClientGoalState status = moveToActionPosition(PROBE, side, level, block);

  ROS_WARN("TEST 0");
  debugBreak();

  /* Probe the block */
  status = executeProbingAction();
  // TODO: The subroutine monitoring the force readings will have to somehow signal here
  //       for decision to continue or to try next block

  ROS_WARN("TEST 1");
  debugBreak();

  /* Return to up position */
  status = moveToHomePosition(side);

  ROS_WARN("TEST 2");
  debugBreak();

  /* Move arm to range finding position on the other side */
  int other_side = (side + 2) % 4;
  status = moveToActionPosition(RANGE_FINDER, other_side, level, block);

  ROS_WARN("TEST 3");
  debugBreak();

  /* Use range finder to find center of the block */
  status = executeRangeFindingAction();

  ROS_WARN("TEST 4");
  debugBreak();

  /* Find center of the block and compensate accordingly */
  // TODO
  // tf::Transform compensation_transform = ...

  /* Return to up position */
  //status = moveToHomePosition(other_side);

  //ROS_WARN("TEST 5");
  //debugBreak();

  /* Move arm to gripping position */
  status = moveToActionPosition(GRIPPER, other_side, level, block);

  ROS_WARN("TEST 6");
  debugBreak();

  /* Grip the block and pull it out */
  status = executeGrippingAction(GRIPPER_CLOSE_NARROW); // Gripping action will auto return to home position

  ROS_WARN("TEST 7");
  debugBreak();

  /* Return to up position */
  status = moveToHomePosition(other_side);

  ROS_WARN("TEST 8");
  debugBreak();

  /* Move arm to grip change position */
  //status = moveToGripChangePosition();
  //ROS_INFO("Moving to grip change position returned with result %s", status.toString().c_str() );

  //ROS_WARN("TEST 9");
  //debugBreak();

  /* Change the grip fron short side to long side */
  status = executeGripChangeAction();
  ROS_INFO("Grip changing returned with result %s", status.toString().c_str() );

  //ROS_WARN("TEST 10");
  debugBreak();

  /* Return to up position */
  moveToHomePosition(5);

  ROS_WARN("TEST 11");
  debugBreak();

  /* Move to block placing position */
  moveToPlaceBlockPosition();

  ROS_WARN("TEST 12");
  debugBreak();

  /* Place the block */
  executePlaceBlockAction();

  ROS_WARN("TEST 13");
  debugBreak();

  /* Return to up position */
  moveToHomePosition(5);

  ROS_WARN("TEST DONE");
}

void TrajCtrl::debugTestFunctions()
{
  top_status_ = std::vector<int> {1, 1, 1};

  moveToHomePosition(5);

  debugBreak();

  //executeGripChangeAction();
  moveToPlaceBlockPosition();

  ROS_WARN("TEST 12");
  debugBreak();

  /* Place the block */
  executePlaceBlockAction();

  ROS_WARN("TEST 13");

  debugBreak();

  moveToHomePosition(5);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "jenga_ur5_trajectory_control");

  ros::NodeHandle nh;
  TrajCtrl trajectory_control(&nh);

  ros::spinOnce();

  trajectory_control.debugTestFunctions();

  ros::spin();

  return 0;
}
