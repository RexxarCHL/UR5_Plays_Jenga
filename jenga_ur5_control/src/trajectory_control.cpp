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
  top_orientation_ = true;
  //top_orientation_ = false;
  top_status_ = std::vector<int> {0, 0, 0};

  //joint_state_.position = std::vector<double>(0.0, 6);

  is_busy_ = false;
  is_probing_ = false;
  is_range_finding_ = false;

  std::fill(compensation_result_.begin(), compensation_result_.end(), tf::Vector3(0, 0, 0));

  initializeSubscriber();
  initializePublisher();
  initializeServiceClient();
  initializeActionClient();

  ros::spinOnce(); // ensure callbacks work properly

  prev_tf_tower_ = retrieveTransform("roadmap_tower");

  ROS_INFO("Connecting to end effector...");
  publishToolCommand(GRIPPER_OPEN_NARROW);
  blockUntilToolFeedback(GRIPPER_OPEN_NARROW);

  initializeWaypointsAndCompensations();
  teachBlockRestConfigurations();
}

/**
 * Initialize subscribers and link their callbacks 
 */
void TrajCtrl::initializeSubscriber()
{
  ROS_INFO("Initializing subscribers");

  jenga_target_subscriber_ = // Queue size of 1 to prevent extra actions from queuing
      nh_.subscribe<jenga_msgs::JengaTarget>("/jenga/target", 1, &TrajCtrl::jengaTargetCallback, this);
  joint_state_subscriber_ = 
      nh_.subscribe<sensor_msgs::JointState>("/joint_states", 10, &TrajCtrl::jointStateCallback, this);

  // Tool related
  tool_feedback_subscriber_ = 
      nh_.subscribe<jenga_msgs::EndEffectorFeedback>("/tool/feedback", 1, &TrajCtrl::feedbackCallback, this);
  tool_range_subscriber_ = nh_.subscribe<jenga_msgs::Probe>("/tool/range", 300, &TrajCtrl::rangeCallback, this);
      //nh_.subscribe<sensor_msgs::Range>("/tool/range", 10, &TrajCtrl::rangeCallback, this);
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
  target_result_publisher_ = nh_.advertise<jenga_msgs::JengaTargetResult>("/jenga/result", 1);
  ar_tower_tracking_publisher_ = nh_.advertise<std_msgs::Bool>("/jenga/tracking", 3);
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

void TrajCtrl::initializeWaypointsAndCompensations()
{  
  /* Get which side to play on */
  std::array<tf::Transform, 4> tf_side;
  std::array<double, 4> angles, distances;
  double min_distance = 999999.99;
  double delta = 0.001;
  int min_index = -1;
  for (int i = 0; i < 4; ++i)
  {
    tf_side[i] = retrieveTransform(SIDE_FRAME_NAMES_[i]);
    tf::Vector3 p = tf_side[i].getOrigin();
    double x = p.getX();
    double y = p.getY();
    angles[i] = std::atan2(y, x);
    distances[i] = std::sqrt(x*x + y*y);

    if (distances[i] < min_distance)
    {
      min_distance = distances[i];
      min_index = i;
    }
  }

  // Do another pass to check for equal distances, albeit a very small chance
  for (int i = 0; i < 4; ++i)
    if (i != min_index && distances[i] < min_distance + delta && distances[i] > min_distance - delta)
    {
      // In case of equal distances, choose the one with smaller angle
      if(angles[i] < angles[min_index])
        min_index = i;
    }

  // Calculate the sides to play on currently
  int this_side = (min_index + 1) % 2;
  //int other_side = (side + 2) % 4;
  std::array<int, 2> sides {{this_side , (this_side + 2) % 4}};

  ROS_WARN("Initializing inverse configurations and compensations on side %d and %d", sides[0], sides[1]);
  ROS_WARN("Put the JENGA tower on the paper now.");
  ROS_WARN("Please be aware of the surroundings and put a hand on E-STOP.");
  debugBreak();

  trajectory_msgs::JointTrajectory trajectory; // Initialize new trajectory
  trajectory.joint_names = UR_JOINT_NAMES_;
  std::vector<double> zero_vector{0, 0, 0, 0, 0, 0};
  control_msgs::FollowJointTrajectoryGoal goal;
  trajectory_msgs::JointTrajectoryPoint joints_current;
  for (int i = 0; i < 2; i++)
  {
    this_side = sides[i];
    ROS_INFO("Moving to side %d", this_side);
    //debugBreak();

    playing_side_ = this_side;

    /* Lookup transform on probing side */
    tf::Transform tf_above, tf_side;
    tf_above = compensateEELinkToGripper( retrieveTransform(ABOVE_FRAME_NAMES_[this_side]) );
    tf_side = compensateEELinkToRangeFinder( retrieveTransform(SIDE_FRAME_NAMES_[this_side]) );

    // DEBUG: show the frames
    tf_broadcaster_.sendTransform(
        tf::StampedTransform(tf_above, ros::Time::now(), "base_link", "tf_above_ee_link"));
    tf_broadcaster_.sendTransform(
        tf::StampedTransform(tf_side, ros::Time::now(), "base_link", "tf_side_ee_link"));

    /* Call inverse kinematics to get possible configurations */
    std::vector<TrajCtrl::Configuration> inv_configs_above, inv_configs_side;
    inv_configs_above = getInverseConfigurations(tf_above);
    inv_configs_side = getInverseConfigurations(tf_side);

    /* Eliminate configurations to only one per waypoint */
    TrajCtrl::Configuration config_above, config_side;
    // Eliminate configurations for roadmap_side using heuristic method
    config_side = eliminateConfigurations(inv_configs_side, tf_side);
    // Eliminate configurations for roadmap_above by least difference to roadmap_side
    config_above = pickMinimumEffortConfiguration(inv_configs_above, config_side);

    // Eliminate excess wrist3 joint movement
    double delta = 0.001;
    double wrist_3_difference = std::abs(config_side[WRIST_3_JOINT] - config_above[WRIST_3_JOINT]);
    if (wrist_3_difference < 2*M_PI + delta && wrist_3_difference > 2*M_PI - delta )
    {
      // wrist 3 rotates 2pi, set two joint values to be the same
      config_above[WRIST_3_JOINT] = config_side[WRIST_3_JOINT];
    }

    /* Drive the arm to the waypoints sequentially */
    // All actions start with current position
    joints_current.positions = getCurrentJointState().position;
    joints_current.velocities = zero_vector;
    joints_current.time_from_start = ros::Duration(0.0); // Start immediately
    trajectory.points.push_back(joints_current);

    ROS_INFO("Point 1:");
    debugPrintJoints(joints_current.positions);

    // Then drive to above position
    trajectory_msgs::JointTrajectoryPoint joints_above;
    joints_above.positions = config_above;
    joints_above.velocities = zero_vector;
    joints_above.time_from_start = ros::Duration(5.0); // TODO: tune this time
    trajectory.points.push_back(joints_above);
    ROS_INFO("Point 2:");
    debugPrintJoints(joints_above.positions);

    
    // Then drive to side position
    trajectory_msgs::JointTrajectoryPoint joints_side;
    joints_side.positions = config_side;
    joints_side.velocities = zero_vector;
    joints_side.time_from_start = ros::Duration(10.0);
    trajectory.points.push_back(joints_side);
    ROS_INFO("Point 3:");
    debugPrintJoints(joints_side.positions);

    goal.trajectory = trajectory;
    executeTrajectoryGoal(goal);
    trajectory.points.clear();

    // If the user had not emergency stop and kill the program by now, this configuration is probably OK
    
    // Store this configuration for future reference
    auto it = stored_configurations_.find(ABOVE_FRAME_NAMES_[this_side]);
    if (it != stored_configurations_.end())
      stored_configurations_.erase(it);
    stored_configurations_.insert( std::pair<std::string, Configuration>(ABOVE_FRAME_NAMES_[this_side], config_above) );

    ROS_INFO("Executing range finding action to find positional compensation for the tower");
    executeRangeFindingAction(false);
    calculateDistance();
    calculateCompensation();

    ROS_INFO("Moving back home");
    moveToHomePosition(this_side);
  }
  
  ROS_INFO("Waypoint and compensations initialized");
  //std::fill(compensation_result_.begin(), compensation_result_.end(), tf::Vector3(0, 0, 0));
}

void TrajCtrl::teachBlockRestConfigurations()
{
  ROS_INFO("Checking drop configuration...");
  tf::Transform tf_block_drop = retrieveTransform("roadmap_block_drop");
  tf::Transform tf_block_pickup = retrieveTransform("roadmap_block_pickup");
  tf::Transform tf_block_rest = retrieveTransform("roadmap_block_rest");

  tf_block_drop = compensateEELinkToGripper(tf_block_drop);
  tf_block_pickup = compensateEELinkToGripper(tf_block_pickup);
  tf_block_rest = compensateEELinkToGripper(tf_block_rest);

  // Get a suitable configuration for target transforms
  TrajCtrl::Configuration config_pickup = eliminateConfigurations(getInverseConfigurations(tf_block_pickup), tf_block_pickup);
  TrajCtrl::Configuration config_rest = pickMinimumEffortConfiguration(getInverseConfigurations(tf_block_rest), config_pickup);
  TrajCtrl::Configuration config_drop = eliminateConfigurations(getInverseConfigurations(tf_block_drop), tf_block_drop);

  // Initialize new trajectory
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names = UR_JOINT_NAMES_;
  std::vector<double> zero_vector{0, 0, 0, 0, 0, 0};
  control_msgs::FollowJointTrajectoryGoal goal;

  // Start with current position
  trajectory_msgs::JointTrajectoryPoint joints_current;
  joints_current.positions = getCurrentJointState().position;
  joints_current.velocities = zero_vector;
  joints_current.time_from_start = ros::Duration(0.0);
  trajectory.points.push_back(joints_current);

  // Move to drop location
  trajectory_msgs::JointTrajectoryPoint joints_drop;
  joints_drop.positions = config_drop;
  joints_drop.velocities = zero_vector;
  joints_drop.time_from_start = ros::Duration(6.0);
  trajectory.points.push_back(joints_drop);

  // Send the trajectory
  goal.trajectory = trajectory;
  executeTrajectoryGoal(goal);
  ROS_INFO("Moving to drop position done");

  ROS_INFO("You can manually move the robot to an appropriate drop position");
  debugBreak();
  ros::spinOnce();
  config_drop = getCurrentJointState().position;

  /* Open gripper to drop the block */
  ROS_INFO("Opening gripper");
  publishToolCommand(GRIPPER_OPEN_WIDE);
  
  // Wait until the gripper is closed
  blockUntilToolFeedback(jenga_msgs::EndEffectorFeedback::ACK_GRIPPER_OPENED);

  /* Move to block pickup position */
  // Reset the trajectory points
  trajectory.points.clear();

  // Start with current position
  joints_current.positions = getCurrentJointState().position;
  joints_current.time_from_start = ros::Duration(0.0);
  trajectory.points.push_back(joints_current);

  // Move to the position above the block
  trajectory_msgs::JointTrajectoryPoint joints_pickup;
  joints_pickup.positions = config_pickup;
  joints_pickup.velocities = zero_vector;
  joints_pickup.time_from_start = ros::Duration(5.0);
  trajectory.points.push_back(joints_pickup);
  // Send the trajectory
  goal.trajectory = trajectory; 
  executeTrajectoryGoal(goal);

  //debugBreak();

  trajectory.points.clear();
  // Start with current position
  joints_current.positions = getCurrentJointState().position;
  joints_current.time_from_start = ros::Duration(0.0);
  trajectory.points.push_back(joints_current);

  trajectory_msgs::JointTrajectoryPoint joints_rest;
  joints_rest.positions = config_rest;
  joints_rest.velocities = zero_vector;
  joints_rest.time_from_start = ros::Duration(5.0);
  trajectory.points.push_back(joints_rest);

  // Send the trajectory
  goal.trajectory = trajectory; 
  executeTrajectoryGoal(goal);

  ROS_INFO("You can manually move the robot to an appropriate pickup position");
  debugBreak();
  ros::spinOnce();
  config_rest = getCurrentJointState().position;

  /* Move to the position above the dropped block */
  // Reset the trajectory points
  trajectory.points.clear();

  // Start with current position
  joints_current.positions = getCurrentJointState().position;
  joints_current.time_from_start = ros::Duration(0.0);
  trajectory.points.push_back(joints_current);

  // Move to the position above the block
  joints_pickup.time_from_start = ros::Duration(5.0);
  trajectory.points.push_back(joints_pickup);

  // Send the trajectory
  goal.trajectory = trajectory; 
  executeTrajectoryGoal(goal);

  // If the user had not emergency stop and kill the program by now, this configuration is probably OK
  // Store this configuration for future reference
  stored_configurations_.insert( std::pair<std::string, Configuration>("roadmap_block_drop", config_drop) );
  stored_configurations_.insert( std::pair<std::string, Configuration>("roadmap_block_pickup", config_pickup) );
  stored_configurations_.insert( std::pair<std::string, Configuration>("roadmap_block_rest", config_rest) );

  /* Open gripper to drop the block */
  ROS_INFO("Reset gripper");
  publishToolCommand(GRIPPER_OPEN_NARROW);
  
  // Wait until the gripper is closed
  blockUntilToolFeedback(jenga_msgs::EndEffectorFeedback::ACK_GRIPPER_OPENED);

  ROS_INFO("Moving back home");
  moveToHomePosition(5);
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

  //for(int i = 0; i < 6; ++i)
  //  std::cout << joint_state_.name[i] << ": " << joint_state_.position[i] << std::endl;
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
void TrajCtrl::jengaTargetCallback(const jenga_msgs::JengaTarget::ConstPtr& target_block)
{
  /* Received candidate blocks */ 
  // TODO: Modify message to receive blocks instead of just one block
  ROS_INFO("Received jenga target block: side%d, level%d, block#%d", 
           target_block->side, target_block->level, target_block->block);
  if(is_busy_) // Block arm driving action from executing until the previous action is done
  {
    ROS_ERROR("Jenga target block received when robot is busy!");
    return;
  }

  is_busy_ = true; // Lock future action requests

  if (checkTowerLocation())
  {
    char c;
    ROS_WARN("Tower location seemed to be changed. Do you want to run waypoint initialization?(y/N) ");
    std::cin >> c;
    std::cin.ignore(INT_MAX, '\n');

    ros::spinOnce();
    if (c == 'y' || c == 'Y')
      initializeWaypointsAndCompensations();
  }

  /* Pick a block */
  // TODO: After modifying the message, pick a block sequentially from the message
  int side = target_block->side;
  int level = target_block->level;
  int block = target_block->block;

  bool result = playBlock(side, level, block);
  
  publishTargetResult(*target_block, result);

  is_busy_ = false; // Release the lock
}

/**
 * Report the result of a target block back to the player
 */
void TrajCtrl::publishTargetResult(jenga_msgs::JengaTarget target_block, bool result)
{
  // Initialize the message
  jenga_msgs::JengaTargetResult target_result;
  target_result.header.stamp = ros::Time::now();
  target_result.target = target_block;
  target_result.result = result;

  // Send the result
  target_result_publisher_.publish(target_result);
}

/**
 * Play a block
 */
bool TrajCtrl::playBlock(int side, int level, int block)
{
  // Stop tracking ar tower location
  std_msgs::Bool msg;
  msg.data = false;
  ar_tower_tracking_publisher_.publish(msg);

  playing_side_ = side;
  playing_level_ = level;
  playing_block_ = block;

  /* Move arm to probing position */
  moveToActionPosition(PROBE, side, level, block);

  ROS_WARN("Moved to probing position. Exectue probing is next...");
  //debugBreak();

  /* Probe the block */
  actionlib::SimpleClientGoalState status = executeProbingAction();

  ROS_WARN("Probing done. Moving back to home position is next...");
  //debugBreak();

  ROS_WARN("Branching...");
  //debugBreak();

  /* See if the probing action succeed */
  if (status != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
  {
    ROS_WARN("Action did not succeed. Returning false is next...");
    ros::Duration(3.0).sleep();
    return false; // Probing failed: the block can not be safely removed.
  }

  ROS_WARN("Action succeed. Returning home...");
  //debugBreak()
  /* Return to up position */
  moveToHomePosition(side);


  /* Move arm to range finding position on the other side */
  int other_side = (side + 2) % 4;
  playing_side_ = other_side;
  moveToActionPosition(RANGE_FINDER, other_side, level, block);

  ROS_WARN("Moved to range finding position. Execute action is next...");
  //debugBreak();

  /* Use range finder to find center of the block */
  executeRangeFindingAction();

  ROS_WARN("Executed range finding action. Move to gripping position is next...");
  //debugBreak();

  /* Find center of the block and compensate accordingly */
  // TODO
  // tf::Transform compensation_transform = ...
  calculateCompensation();

  /* Move arm to gripping position */
  moveToActionPosition(GRIPPER, other_side, level, block);

  ROS_WARN("Moved to gripping position. Gripping action is next...");
  //debugBreak();

  /* Grip the block and pull it out */
  executeGrippingAction(GRIPPER_CLOSE_NARROW); // Gripping action will auto return to home position

  ROS_WARN("Gripped. Return home is next...");
  //debugBreak();

  /* Return to up position */
  moveToHomePosition(other_side);

  ROS_WARN("Moved back to home position is done. Grip change is next...");
  //debugBreak();

  /* Change the grip fron short side to long side */
  executeGripChangeAction();

  ROS_WARN("Grip change is done. Return home is next...");
  //debugBreak();

  /* Return to up position */
  moveToHomePosition(5);

  ROS_WARN("Moving back to home position is done. Move to block placing position is next...");
  //debugBreak();

  /* Move to block placing position */
  moveToPlaceBlockPosition();

  ROS_WARN("Moved to block placing position. Placing block is next...");
  //debugBreak();

  /* Place the block */
  executePlaceBlockAction();

  ROS_WARN("Placed the block. Returning home is next...");
  //debugBreak();

  /* Return to up position */
  moveToHomePosition(5);

  // Resume tracking
  msg.data = true;
  ar_tower_tracking_publisher_.publish(msg);


  ROS_WARN("Done! Returning true...");
  debugBreak();

  return true;
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

  return rv;
}

/**
 * Cherry pick some suitable configurations for Jenga playing
 */
TrajCtrl::Configuration TrajCtrl::eliminateConfigurations(
    std::vector<TrajCtrl::Configuration> configurations, tf::Transform target_transform)
{
  ROS_INFO("Eliminating configurations");

  //driveToEveryConfig(configurations);

  std::vector<TrajCtrl::Configuration> rv;
  
  for(auto config: configurations)
  {
    ROS_INFO("This configuration:");
    debugPrintJoints(config);

    if (config[SHOULDER_LIFT_JOINT] > 0)
    {
      ROS_INFO("--REJECTED: lift joint > 0");
      continue;
    }

    if (config[SHOULDER_PAN_JOINT] * config[ELBOW_JOINT] < 0)
    {
      ROS_INFO("--REJECTED: pan * elbow < 0");
      continue;
    }

    if (config[SHOULDER_LIFT_JOINT] * config[WRIST_1_JOINT] < 0)
    {
      ROS_INFO("--REJECTED: lift * wirst 1 < 0");
      continue;
    }

    if (config[WRIST_1_JOINT] > 0.1)
    {
      ROS_INFO("--REJECTED: wrist 1 > 0.1");
      continue;
    }

    // If reached here, this config is probably good
    ROS_INFO("--OK!");
    rv.push_back(config);
  }

  ROS_INFO("Found %lu probably good solutions", rv.size());
  if (!rv.size())
    ROS_WARN("NO GOOD SOLUTIONS FOUND!");

  if (rv.size() > 1)
    return tieBreak(rv, target_transform);
  else
    return rv[0];
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
/**
 * Compare the arctan angles on xy-plane to determine the proper position for shoulder_pan_joint.
 */
TrajCtrl::Configuration TrajCtrl::tieBreak(std::vector<TrajCtrl::Configuration> configs, tf::Transform target_transform) 
{
  ROS_INFO("Tie breaking configurations...");
  ROS_ERROR_COND(configs.size() != 2, "The size of configs is NOT 2");

  /* Get arctan angles of tower and target based on its x-y location */
  tf::Transform tf_tower = retrieveTransform("roadmap_tower"); // Get the tower location

  tf::Vector3 translation_tower = tf_tower.getOrigin();
  tf::Vector3 translation_target = target_transform.getOrigin();

  double rotation_tower, rotation_target;
  rotation_tower = std::atan2( translation_tower.getY(), translation_tower.getX() );
  rotation_target = std::atan2( translation_target.getY(), translation_target.getX() );

  /* Do angle comparison */
  Configuration rv;
  if (rotation_target > rotation_tower)
    rv = configs[0][SHOULDER_PAN_JOINT] < 0 ? configs[0] : configs[1]; // Select pan < 0 configuration
  else // rotation_target <= rotation_tower
    rv = configs[0][SHOULDER_PAN_JOINT] > 0 ? configs[0] : configs[1]; // Select pan > 0 configuration

  ROS_INFO("Selected: ");
  debugPrintJoints(rv);
  return rv;
}

// Check if the configuration is already calculated. If not, calculate configuration
TrajCtrl::Configuration TrajCtrl::checkStoredConfigurations(std::string frame_name, Configuration reference_config)
{
  auto config = stored_configurations_.find(frame_name);
  if (config != stored_configurations_.end())
  {
    ROS_INFO("Configuration found for %s!", frame_name.c_str());
    debugPrintJoints(config->second);
    //debugBreak();
    return config->second; // Found a configuration
  }

  ROS_WARN("No configuration for %s found. Calculating...", frame_name.c_str());
  //debugBreak();

  /* Lookup transform on the route */
  tf::Transform tf_target = compensateEELinkToGripper(retrieveTransform(frame_name));

  /* Call inverse kinematics to get possible configurations */
  std::vector<TrajCtrl::Configuration> inv_configs_target;
  inv_configs_target = getInverseConfigurations(tf_target);

  /* Eliminate configurations to only one per waypoint */
  TrajCtrl::Configuration config_target;
  // Eliminate configurations for roadmap_above using heuristic method
  //config_above = eliminateConfigurations(inv_configs_above);
  config_target = pickMinimumEffortConfiguration(inv_configs_target, reference_config);

  // Eliminate excess wrist3 joint movement
  double delta = 0.001;
  double wrist_3_difference = std::abs(config_target[WRIST_3_JOINT] - reference_config[WRIST_3_JOINT]);
  if (wrist_3_difference < 2*M_PI + delta && wrist_3_difference > 2*M_PI - delta )
  {
    // wrist 3 rotates 2pi, set two joint values to be the same
    config_target[WRIST_3_JOINT] = reference_config[WRIST_3_JOINT];
  }

  stored_configurations_.insert( std::pair<std::string, TrajCtrl::Configuration>(frame_name, config_target) );

  return config_target;
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
  tf::Transform tf_block(
      tf::Quaternion::getIdentity(), 
      tf::Vector3(x_offset, y_offset, 0.0) + compensation_result_[playing_side_]);

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
  tf::Transform tool0_to_tool_gripper( tf::Quaternion::getIdentity(), tf::Vector3(0, 0, 0.077646) );

  tf::Transform ee_link_to_tool_gripper = ee_link_to_tool0 * tool0_to_tool_gripper;

  transform = transform * ee_link_to_tool_gripper.inverse();

  return transform;
}
tf::Transform TrajCtrl::compensateEELinkToRangeFinder(tf::Transform transform)
{
  tf::Transform ee_link_to_tool0( tf::Quaternion(-0.5, 0.5, -0.5, 0.5) ); 
  tf::Transform tool0_to_tool_range_finder( tf::Quaternion::getIdentity(), tf::Vector3(0, -0.02942, 0.039234) );

  tf::Transform ee_link_to_tool_range_finder = ee_link_to_tool0 * tool0_to_tool_range_finder;

  transform = transform * ee_link_to_tool_range_finder.inverse();

  return transform;
}
tf::Transform TrajCtrl::compensateEELinkToProbe(tf::Transform transform)
{
  tf::Transform ee_link_to_tool0( tf::Quaternion(-0.5, 0.5, -0.5, 0.5) ); 
  tf::Transform tool0_to_tool_probe( tf::Quaternion(0.0, M_PI/4, 0.0, M_PI/4), tf::Vector3(0.0795, -0.003, 0.026146) );

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
 * Send the trajectory to action server. Does not block for completion.
 */
void TrajCtrl::executeTrajectoryGoalNonblocking(control_msgs::FollowJointTrajectoryGoal trajectory_goal)
{
  ROS_INFO("Sending goal to action server");

  action_client_->sendGoal(trajectory_goal);
}

/**
 * Generate a trajectory goal object that drive the arm from current position to input target_transform.
 */
control_msgs::FollowJointTrajectoryGoal TrajCtrl::generateTrajectory(int side, tf::Transform tf_target)
{
  ROS_INFO("Generating trajectory to side%d", side);

  /* Lookup transform on the route */
  std::string frame_name_above("roadmap_above" + std::to_string(side) );
  tf::Transform tf_above = compensateEELinkToGripper( retrieveTransform(frame_name_above) );

  // DEBUG: show the frames
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(tf_above, ros::Time::now(), "base_link", "tf_above_ee_link"));

  /* Call inverse kinematics to get possible configurations for target */
  std::vector<TrajCtrl::Configuration> inv_configs_target = getInverseConfigurations(tf_target);

  /* Eliminate configurations to only one per waypoint */
  TrajCtrl::Configuration config_above, config_target;
  // Eliminate configurations for roadmap_above using heuristic method
  config_target = eliminateConfigurations(inv_configs_target, tf_target);
  config_above = checkStoredConfigurations(frame_name_above, config_target);

  /* Drive the arm to the waypoints sequentially */
  ROS_INFO("[generateTrajectory] Initializing new trajectory");
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

  ROS_INFO("[generateTrajectory] Point 1:");
  debugPrintJoints(joints_current.positions);

  // Then drive to above position
  trajectory_msgs::JointTrajectoryPoint joints_above;
  joints_above.positions = config_above;
  joints_above.velocities = zero_vector;
  joints_above.time_from_start = ros::Duration(2.0); // TODO: tune this time
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
  double delta = 0.001;
  double wrist_3_difference = std::abs(config_target[WRIST_3_JOINT] - config_above[WRIST_3_JOINT]);
  if (wrist_3_difference < 2*M_PI + delta && wrist_3_difference > 2*M_PI - delta )
  {
    // wrist 3 rotates 2pi, set two joint values to be the same
    config_target[WRIST_3_JOINT] = config_above[WRIST_3_JOINT];
  }

  // Finally, drive to target
  trajectory_msgs::JointTrajectoryPoint joints_target;
  joints_target.positions = config_target;
  joints_target.velocities = zero_vector;
  joints_target.time_from_start = ros::Duration(4.0); // TODO: tune this time
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
  tf::Transform tf_above = compensateEELinkToGripper( retrieveTransform(frame_name_above) );

  // DEBUG: show the frames
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(tf_above, ros::Time::now(), "base_link", "tf_above_ee_link"));

  /* Get the inverse transform */
  TrajCtrl::Configuration config_above = checkStoredConfigurations(frame_name_above, getCurrentJointState().position);

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

  double delta = 0.001;
  double wrist_3_difference = std::abs(joints_current.positions[WRIST_3_JOINT] - config_above[WRIST_3_JOINT]);
  if (wrist_3_difference < 2*M_PI + delta && wrist_3_difference > 2*M_PI - delta )
  {
    // wrist 3 rotates 2pi, set two joint values to be the same
    config_above[WRIST_3_JOINT] = joints_current.positions[WRIST_3_JOINT];
  }

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
  joints_up.time_from_start = ros::Duration( (side < 5)? 4.0 : 3.0 );
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
 * NOTE: The return state represents the result of the probing action, 
 *       SUCCEED means success and PREEMPTED means failed
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
  //tf::Transform tf_target( tf::Quaternion::getIdentity(), tf::Vector3(0, 0, 0.09) );
  tf::Transform tf_target( tf::Quaternion::getIdentity(), tf::Vector3(0, 0, distance_to_tower_[playing_side_] + 0.05) );
  ROS_INFO("distance = %f", distance_to_tower_[playing_side_] + 0.05);
  //debugBreak();

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

  /* Signal tool to turn on the probe */
  publishToolCommand(jenga_msgs::EndEffectorControl::PROBE_ON);
  
  // Wait until the probe is on
  bool feedback_result = blockUntilToolFeedback(jenga_msgs::EndEffectorControl::PROBE_ON);
  ROS_INFO("[executeProbingAction] Tool feedback: %d", feedback_result);

  /* Send the trajectory */
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  executeTrajectoryGoalNonblocking(goal); 
  // It actually does not matter if probing is successful or not. We always want the probe to return to starting config
  // The subroutine that signals success or fail is responsible of letting others know the result.

  /* Let the callback monitor the probe topic for force readings */
  is_probing_ = true; // Enable callback function for /tool/probe to do its job
  while (!action_client_->getState().isDone())
    ros::spinOnce(); // Check and execute callbacks
  is_probing_ = false; // Let callback function for /tool/probe return immediately
  // Register the result of probing action
  actionlib::SimpleClientGoalState status = action_client_->getState();

  /* Signal tool to turn off the probe */
  publishToolCommand(jenga_msgs::EndEffectorControl::PROBE_OFF);
  
  // Wait until the probe is off
  feedback_result = blockUntilToolFeedback(jenga_msgs::EndEffectorControl::PROBE_OFF);
  ROS_INFO("[executeProbingAction] Tool feedback: %d", feedback_result);

  /* Move back to starting position */
  // Clear the trajectory
  trajectory.points.clear();

  // Start with current position
  joints_current.positions = getCurrentJointState().position;
  trajectory.points.push_back(joints_current);
  ROS_INFO("[executeProbingAction] Return, Point 1:");
  debugPrintJoints(joints_current.positions);

  // Then drive back to start position
  joints_target.positions = config_start;
  joints_target.time_from_start = ros::Duration(1.0);
  trajectory.points.push_back(joints_target);
  ROS_INFO("[executeProbingAction] Return, Point 2:");
  debugPrintJoints(joints_target.positions);

  /* Send the trajectory */
  goal.trajectory = trajectory;
  executeTrajectoryGoal(goal);

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
    tf_target = tf::Transform( tf::Quaternion::getIdentity(), tf::Vector3(0, 0, 0.06) );
  }
  else // close long
  {
    // Calculate the offset
    //tf::Transform tf_block_pickup = retrieveTransform("roadmap_block_pickup");
    //tf::Transform tf_block_rest = retrieveTransform("roadmap_block_rest");

    //double offset = tf_block_pickup.getOrigin().getZ() - tf_block_rest.getOrigin().getZ();
    tf_target = tf::Transform( 
        tf::Quaternion::getIdentity(), 
        tf::Vector3(0, 0, 0.060) );
        //tf::Vector3(0, 0, above_offset - GRIPPER_FRAME_TIP_OFFSET_) );
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
  joints_target.time_from_start = ros::Duration(1.0);
  trajectory.points.push_back(joints_target);
  ROS_INFO("[executeGrippingAction] Point 2:");
  debugPrintJoints(joints_target.positions);

  /* Send the trajectory */
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  executeTrajectoryGoal(goal); 

  //debugBreak();

  /* Close gripper */
  // Prepare and send the command message
  publishToolCommand(mode);
  
  // Wait until the gripper is closed
  bool feedback_result = blockUntilToolFeedback(mode);
  ROS_INFO("[executeGrippingAction] Tool feedback: %d", feedback_result);

  /* Move back to starting position */
  // Clear the trajectory
  trajectory.points.clear();

  // Start with current position
  joints_current.positions = getCurrentJointState().position;
  trajectory.points.push_back(joints_current);
  ROS_INFO("[executeGrippingAction] Return, Point 1:");
  debugPrintJoints(joints_current.positions);

  // Then drive back to start position
  joints_target.positions = config_start;
  joints_target.time_from_start = ros::Duration(1.5);
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
actionlib::SimpleClientGoalState TrajCtrl::executeRangeFindingAction(bool mode) // default = true
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

  double offset = mode? 0.035 : 0.06;
  tf::Transform tf_left( tf::Quaternion::getIdentity(), tf::Vector3(offset, 0, 0) );
  tf::Transform tf_right( tf::Quaternion::getIdentity(), tf::Vector3(-offset, 0, 0) );

  tf_left = compensateEELinkToRangeFinder(tf_range_finder * tf_left);
  tf_right = compensateEELinkToRangeFinder(tf_range_finder * tf_right);

  /* Get the configuration for target transforms */
  TrajCtrl::Configuration config_start = getCurrentJointState().position; // Save current configuration for pulling out
  TrajCtrl::Configuration config_left = 
      pickMinimumEffortConfiguration( getInverseConfigurations(tf_left), config_start );
  TrajCtrl::Configuration config_right = 
      pickMinimumEffortConfiguration( getInverseConfigurations(tf_right), config_start );

  // Eliminate excess wrist3 joint movement
  std::vector<TrajCtrl::Configuration*> configs {&config_left, &config_right};
  double delta = 0.001;
  double wrist_3_difference;
  for (auto config: configs)
  {
    wrist_3_difference = std::abs((*config)[WRIST_3_JOINT] - config_start[WRIST_3_JOINT]);
    if (wrist_3_difference < 2*M_PI + delta && wrist_3_difference > 2*M_PI - delta )
    {
      // wrist 3 rotates 2pi, set two joint values to be the same
      (*config)[WRIST_3_JOINT] = config_start[WRIST_3_JOINT];
    }
  }

  /* Move start -> left -> right -> start */

  /* Initialize trajectory */
  ROS_INFO("[executeRangeFindingAction] Initializing trajectory");
  trajectory_msgs::JointTrajectory trajectory; // Initialize new trajectory
  trajectory.joint_names = UR_JOINT_NAMES_;
  std::vector<double> zero_vector{0, 0, 0, 0, 0, 0};
  double time_required = 0.5;

  ROS_INFO("configurations:");
  debugPrintJoints(config_start);
  debugPrintJoints(config_left);
  debugPrintJoints(config_right);

  /* Move to left position first */
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
  joints_left.time_from_start = ros::Duration(time_required);
  trajectory.points.push_back(joints_left);

  // Send the trajectory
  ROS_INFO("[executeRangeFindingAction] Moving to left position");
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  executeTrajectoryGoal(goal);


  /* Scan from left to right and collect range data */
  trajectory.points.clear(); // Clear trajectory points
  
  // Start from current position
  joints_left.positions = getCurrentJointState().position;
  joints_left.time_from_start = ros::Duration(0.0);
  trajectory.points.push_back(joints_left);

  // Drive to right position
  trajectory_msgs::JointTrajectoryPoint joints_right;
  joints_right.positions = config_right;
  joints_right.velocities = zero_vector;
  joints_right.time_from_start = ros::Duration(mode? 10.0: 15.0);
  trajectory.points.push_back(joints_right);

  // Signal the tool to start sending range finder data
  publishToolCommand(jenga_msgs::EndEffectorControl::RANGE_ON);
  
  // Wait until the tool says range finder is on
  bool feedback_result = blockUntilToolFeedback(jenga_msgs::EndEffectorControl::RANGE_ON);
  ROS_INFO("[executeRangeFindingAction] Tool feedback: %d", feedback_result);

  // Send the trajectory
  ROS_INFO("[executeRangeFindingAction] Scanning...");
  goal.trajectory = trajectory;
  executeTrajectoryGoalNonblocking(goal);

  // Let callbacks process until the action is done
  is_range_finding_ = true; // Enable callback function for /tool/range to do its job
  while (!action_client_->getState().isDone())
    ros::spinOnce(); // Check and execute callbacks
  is_range_finding_ = false; // Let callback function for /tool/range return immediately

  // Signal the tool to stop sending range finder message
  publishToolCommand(jenga_msgs::EndEffectorControl::RANGE_OFF);
  
  // Wait until the tool says range finder is off
  feedback_result = blockUntilToolFeedback(jenga_msgs::EndEffectorControl::RANGE_OFF);
  ROS_INFO("[executeRangeFindingAction] Tool feedback: %d", feedback_result);


  /* Drive back to original position */
  trajectory.points.clear(); // Clear the trajectory

  joints_right.positions = getCurrentJointState().position;
  joints_right.time_from_start = ros::Duration(0.0);
  trajectory.points.push_back(joints_right);

  // Drive back to start position again
  joints_start.time_from_start = ros::Duration(time_required);
  trajectory.points.push_back(joints_start);

  // Send trajectory
  ROS_INFO("[executeRangeFindingAction] Moving back to original position");
  goal.trajectory = trajectory;
  executeTrajectoryGoal(goal);
  
  actionlib::SimpleClientGoalState status = action_client_->getState();

  ROS_INFO("[executeRangeFindingAction] Range finding action done, with status %s", status.toString().c_str() );
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
  tf::Transform tf_block_pickup = retrieveTransform("roadmap_block_pickup");

  tf_block_drop = compensateEELinkToGripper(tf_block_drop);
  tf_block_pickup = compensateEELinkToGripper(tf_block_pickup);

  tf_broadcaster_.sendTransform(
      tf::StampedTransform(tf_block_drop, ros::Time::now(), "base_link", "tf_block_drop_ee_link"));

  /* Move from current position (assumed at home) to drop location */
  // These configurations are precalculated at initialization stage
  TrajCtrl::Configuration config_pickup = checkStoredConfigurations("roadmap_block_pickup");
  TrajCtrl::Configuration config_drop = checkStoredConfigurations("roadmap_block_drop");
  TrajCtrl::Configuration config_rest = checkStoredConfigurations("roadmap_block_rest");

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
  /*
  trajectory_msgs::JointTrajectoryPoint joints_intermediate;
  joints_intermediate.positions = joints_current.positions;
  joints_intermediate.positions[0] = config_drop[0];
  joints_intermediate.positions[5] = config_drop[5];
  joints_intermediate.velocities = zero_vector;
  joints_intermediate.time_from_start = ros::Duration(3.0);
  trajectory.points.push_back(joints_intermediate);
  */

  // Move to drop location
  trajectory_msgs::JointTrajectoryPoint joints_drop;
  joints_drop.positions = config_drop;
  joints_drop.velocities = zero_vector;
  joints_drop.time_from_start = ros::Duration(3.0);
  trajectory.points.push_back(joints_drop);

  // Send the trajectory
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  actionlib::SimpleClientGoalState status = executeTrajectoryGoal(goal);
  ROS_INFO("[executeGripChangeAction] Moving to drop position done, with status %s", status.toString().c_str() );

  //debugBreak();

  /* Open gripper to drop the block */
  ROS_INFO("[executeGripChangeAction] Opening gripper");
  publishToolCommand(GRIPPER_OPEN_WIDE);
  
  // Wait until the gripper is closed
  bool feedback_result = blockUntilToolFeedback(GRIPPER_OPEN_WIDE);
  ROS_INFO("[executeGripChangeAction] Tool feedback: %d", feedback_result);

  /* Move to the position above the dropped block */
  // Reset the trajectory points
  trajectory.points.clear();

  // Start with current position
  joints_current.positions = getCurrentJointState().position;
  trajectory.points.push_back(joints_current);

  // Move to the position above the block
  trajectory_msgs::JointTrajectoryPoint joints_pickup;
  joints_pickup.positions = config_pickup;
  joints_pickup.velocities = zero_vector;
  joints_pickup.time_from_start = ros::Duration(1.5);
  trajectory.points.push_back(joints_pickup);

  trajectory_msgs::JointTrajectoryPoint joints_rest;
  joints_rest.positions = config_rest;
  joints_rest.velocities = zero_vector;
  joints_rest.time_from_start = ros::Duration(2.5);
  trajectory.points.push_back(joints_rest);

  // Send the trajectory
  goal.trajectory = trajectory; 
  status = executeTrajectoryGoal(goal);
  ROS_INFO("[executeGripChangeAction] Moving to gripping position done, with status %s", 
      status.toString().c_str() );

  // Close gripper
  ROS_INFO("[executeGripChangeAction] Closing gripper");
  publishToolCommand(GRIPPER_CLOSE_WIDE);
  
  // Wait until the gripper is closed
  feedback_result = blockUntilToolFeedback(GRIPPER_CLOSE_WIDE);
  ROS_INFO("[executeGripChangeAction] Tool feedback: %d", feedback_result);

  trajectory.points.clear();
  // Start with current position
  joints_current.positions = getCurrentJointState().position;
  joints_current.time_from_start = ros::Duration(0.0);
  trajectory.points.push_back(joints_current);

  // Move to the position above the block
  joints_pickup.time_from_start = ros::Duration(1.5);
  trajectory.points.push_back(joints_pickup);

  goal.trajectory = trajectory; 
  status = executeTrajectoryGoal(goal);
  ROS_INFO("[executeGripChangeAction] Moving back to position above the block block done, with status %s", 
      status.toString().c_str() );

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
  tf::Transform tf_target(tf::Quaternion::getIdentity(), tf::Vector3(0, 0, z_offset - GRIPPER_FRAME_TIP_OFFSET_));

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
  joints_target.time_from_start = ros::Duration(1.0);
  trajectory.points.push_back(joints_target);
  ROS_INFO("[executePlaceBlockAction] Point 2:");
  debugPrintJoints(joints_target.positions);

  /* Send the trajectory */
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  executeTrajectoryGoal(goal); 

  /* Open gripper */
  ROS_INFO("[executePlaceBlockAction] Opening gripper");
  publishToolCommand(GRIPPER_OPEN_WIDE);
  
  // Wait until the gripper is closed
  bool feedback_result = blockUntilToolFeedback(GRIPPER_OPEN_WIDE);
  ROS_INFO("[executePlaceBlockAction] Tool feedback: %d", feedback_result);

  /* Move back to starting position */
  // Clear the trajectory
  trajectory.points.clear();

  // Start with current position
  joints_current.positions = getCurrentJointState().position;
  trajectory.points.push_back(joints_current);
  ROS_INFO("[executePlaceBlockAction] Return, Point 1:");
  debugPrintJoints(joints_current.positions);

  // Then drive back to start position
  joints_target.positions = config_start;
  joints_target.time_from_start = ros::Duration(1.0);
  trajectory.points.push_back(joints_target);
  ROS_INFO("[executePlaceBlockAction] Return, Point 2:");
  debugPrintJoints(joints_target.positions);

  /* Send the trajectory */
  goal.trajectory = trajectory;
  actionlib::SimpleClientGoalState status = executeTrajectoryGoal(goal);

  // Reset tool opening size
  publishToolCommand(GRIPPER_OPEN_NARROW);
  feedback_result = blockUntilToolFeedback(GRIPPER_OPEN_NARROW);
  ROS_INFO("[executePlaceBlockAction] Tool feedback: %d", feedback_result);

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
  ROS_INFO("block = %d", block);
  if (top_orientation_)
    tf_target = tf::Transform( 
        tf::Quaternion::getIdentity(), 
        tf::Vector3(compensation_result_[playing_side_].getY(), block * 0.025, 0));
  else
  {
    tf::Quaternion q; q.setRPY(0, 0, M_PI/2); // Rotate +z 90 degrees
    tf_target = tf::Transform( q, tf::Vector3(block * 0.025, 0, 0));// + compensation_result_[playing_side_]);
  }

  tf_target = compensateEELinkToGripper(tf_direct_above * tf_target);

  ROS_INFO("current_level_: %d, block: %d", current_level_, block);

  /* Get the configuration for target transform */
  TrajCtrl::Configuration config_start = getCurrentJointState().position; // Save current configuration for when action is done
  TrajCtrl::Configuration config_direct_above = checkStoredConfigurations("roadmap_direct_above", config_start);
  TrajCtrl::Configuration config_target = 
      pickMinimumEffortConfiguration( getInverseConfigurations(tf_target), config_direct_above );

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

  // Move to direct above waypoint
  trajectory_msgs::JointTrajectoryPoint joints_direct_above;
  joints_direct_above.positions = config_direct_above;
  joints_direct_above.velocities = zero_vector;
  joints_direct_above.time_from_start = ros::Duration(2.5); // Start immediately
  trajectory.points.push_back(joints_direct_above);
  ROS_INFO("[executePlaceBlockAction] Point 1:");
  debugPrintJoints(joints_direct_above.positions);

  // Then drive to target position
  trajectory_msgs::JointTrajectoryPoint joints_target;
  joints_target.positions = config_target;
  joints_target.velocities = zero_vector;
  joints_target.time_from_start = ros::Duration(3.5);
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
    top_status_[0] = 1;

    block = -1; // Set block number
  }

  return block;
}
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

  for (int i = 0; i < 6; ++i)
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

bool TrajCtrl::checkTowerLocation()
{
  ROS_INFO("Checking if tower location is drastically changed");
  tf::Transform tf_tower = retrieveTransform("roadmap_tower");

  tf::Vector3 diff_t = prev_tf_tower_.getOrigin() - tf_tower.getOrigin();
  tf::Quaternion diff_q = prev_tf_tower_.getRotation() - tf_tower.getRotation();

  prev_tf_tower_ = tf_tower;

  const double MAX_Q_DIFF = 0.005;
  const double MAX_P_DIFF = 0.005;

  if (diff_q.length() > MAX_Q_DIFF || diff_t.length() > MAX_P_DIFF)
  {
    // Pose drastically changed; reset counter to learn the new positions
    ROS_WARN("TOWER POSE CHANGED!");
    ROS_WARN("Difference: %f, %f", diff_q.length(), diff_t.length());
    return true;
  }

  return false;
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
bool TrajCtrl::blockUntilToolFeedback(int command_code)
{
  // Spin until flag is raised. Will wait forever
  // Flag is raised when feedbackCallback is called by the subscriber
  ros::Time timeout = ros::Time::now() + ros::Duration(5.0);
  while (!tool_feedback_flag_)
  {
    ros::spinOnce(); // Process callbacks
    ROS_INFO_DELAYED_THROTTLE(5, "Still waiting...");

    if(ros::Time::now() > timeout)
    {
      ROS_ERROR("Waited for too long; republishing command %d", command_code);
      publishToolCommand(command_code);
      timeout = ros::Time::now() + ros::Duration(5.0);
    }
  }

  // Read feedback code and reset flag
  int received_feedback_code = tool_feedback_code_;
  tool_feedback_flag_ = false;

  int expected_feedback_code;
  switch (command_code)
  {
    case GRIPPER_OPEN_WIDE:
    case GRIPPER_OPEN_NARROW:
      expected_feedback_code = jenga_msgs::EndEffectorFeedback::ACK_GRIPPER_OPENED;
    case GRIPPER_CLOSE_WIDE:
    case GRIPPER_CLOSE_NARROW:
      expected_feedback_code = jenga_msgs::EndEffectorFeedback::ACK_GRIPPER_CLOSED;
    case jenga_msgs::EndEffectorControl::PROBE_ON:
      expected_feedback_code = jenga_msgs::EndEffectorFeedback::ACK_PROBE_ON;
    case jenga_msgs::EndEffectorControl::PROBE_OFF:
      expected_feedback_code = jenga_msgs::EndEffectorFeedback::ACK_PROBE_OFF;
    case jenga_msgs::EndEffectorControl::RANGE_ON:
      expected_feedback_code = jenga_msgs::EndEffectorFeedback::ACK_RANGE_ON;
    case jenga_msgs::EndEffectorControl::RANGE_OFF:
      expected_feedback_code = jenga_msgs::EndEffectorFeedback::ACK_RANGE_OFF;
  }

  ROS_INFO("Expected %d, received %d", expected_feedback_code, received_feedback_code);

  return (received_feedback_code == expected_feedback_code);
}
void TrajCtrl::feedbackCallback(const jenga_msgs::EndEffectorFeedback::ConstPtr& msg)
{
  if (tool_feedback_flag_)
    ROS_WARN("Feedback received but flag is true! Overwriting feedback code.");
  
  tool_feedback_code_ = msg->feedback_code;

  if (tool_feedback_code_ < 6)
    tool_feedback_flag_ = true; // Raise flag to signal the receptance of a feedback from tool
  else
    ROS_ERROR("Feedback code >= 6!");
}
// Cancel the goal if force is more than PROBE_FORCE_THRESHOLD_
void TrajCtrl::probeCallback(const jenga_msgs::Probe::ConstPtr& msg)
{
  if (!is_probing_)
    return;

  double force = msg->data;

  if (force > PROBE_FORCE_THRESHOLD_)
    action_client_->cancelGoal();
}
void TrajCtrl::rangeCallback(const jenga_msgs::Probe::ConstPtr& msg)
{
  if (!is_range_finding_)
    return;

  // DEBUG check for dropped message
  int sequence_id = msg->header.seq;
  ROS_INFO("Processing rangeCallback for %d", sequence_id);

  ros::Time time = msg->header.stamp;
  double range = msg->data;
  ROS_INFO("range: %f", range);

  // Record the current tf and range data as a pair, for later analysis
  tf::StampedTransform tf_stamped;
  std::string tf_side_name = "roadmap_side" + std::to_string((playing_side_+2) % 4); // get side tf of "other side" as fixed reference
  bool frame_exists = tf_listener_.waitForTransform(tf_side_name, "tool_range_finder", time, ros::Duration(0.3));

  tf_listener_.lookupTransform(tf_side_name, "tool_range_finder", time, tf_stamped);
  //tf::Transform tf_range_finder(tf_stamped.getBasis(), tf_stamped.getOrigin());

  std::pair<double, double> current_data(tf_stamped.getOrigin().getX(), range);
  range_finder_data_.push_back(current_data);
}

void TrajCtrl::calculateDistance()
{
  ROS_INFO("Calculating distance to tower");
  /* Unzip the data */
  std::vector<double> data;
  for (auto p: range_finder_data_)
    data.push_back(p.second);

  /* Calculate the mid point of the block from the min and max indices */
  std::pair<int, int> peak_indices = getPeaks(rollingMean(diff(rollingMean(data))));
  int peak1_index = peak_indices.first;
  int peak2_index = peak_indices.second;
  ROS_INFO("peaks: %d, %d", peak1_index, peak2_index);

  
  // Give some offset to reach flat area
  peak1_index += 30;
  peak2_index -= 30;
  
  // Cases to cope with the very off chance that range finder data is not long enough
  if (peak1_index > data.size())
    peak1_index = data.size();
  if (peak2_index < 0)
    peak2_index = 0;
  if (peak2_index < peak1_index)
    std::swap(peak1_index, peak2_index);
  
  double sum = 0.0;

  for (int i = peak1_index; i < peak2_index; ++i)
    sum += data[i];

  distance_to_tower_[playing_side_] = sum / (peak2_index - peak1_index + 1) / 100;

  ROS_INFO("Avg distance on side %d: %f", playing_side_, distance_to_tower_[playing_side_]);
}

void TrajCtrl::calculateCompensation()
{
  ROS_INFO("Calculating compensation");

  /* Unzip the data */
  std::vector<double> y, data;
  for (auto p: range_finder_data_)
  {
    y.push_back(p.first);
    data.push_back(p.second);
  }

  // DEBUG: zip the processed data back and store them
  std::vector<double> smoothed_v = rollingMean(data);
  std::vector<double> smoothed_diff = rollingMean(diff(smoothed_v));
  std::vector<std::pair<double, double>> v;
  for (int i = 0; i < smoothed_v.size(); ++i)
  {
    if(i == smoothed_diff.size())
      v.push_back(std::pair<double, double>(smoothed_v[i], 0.0));
    else
      v.push_back(std::pair<double, double>(smoothed_v[i], smoothed_diff[i]));
  }
  saveData(range_finder_data_, "range_finder");
  saveData(v, "compensate_debug");

  /* Calculate the mid point of the block from the min and max indices */
  std::pair<int, int> peak_indices = getPeaks(rollingMean(diff(rollingMean(data))));
  int peak1_index = peak_indices.first;
  int peak2_index = peak_indices.second;
  ROS_INFO("peaks: %d, %d", peak1_index, peak2_index);
  double mid_point;
  if ((peak1_index + peak2_index) % 2) // min + max is even
  {
    int mid_index = (int) (peak1_index + peak2_index) / 2;
    mid_point = y[mid_index];
  }
  else
  {
    float mid = (peak1_index + peak2_index) / 2;
    int index1 = (int) std::floor(mid);
    int index2 = (int) std::ceil(mid);

    mid_point = (y[index1] + y[index2]) / 2;
  }

  /* Get bias between side waypoint and current range finder position */
  ros::Time now = ros::Time::now();
  tf::StampedTransform tf_stamped;
  std::string tf_side_name = "roadmap_side" + std::to_string((playing_side_+2) % 4); // get side tf of "other side" as fixed reference
  bool frame_exists = tf_listener_.waitForTransform(tf_side_name, "tool_range_finder", now, ros::Duration(0.3));

  tf_listener_.lookupTransform(tf_side_name, "tool_range_finder", now, tf_stamped);
  double y_bias = tf_stamped.getOrigin().getX();

  /* Calculate and store compensation distance */
  ROS_INFO("mid: %f, bias: %f", mid_point, y_bias);
  mid_point -= y_bias;
  ROS_INFO("Compensation: %f", mid_point);
  compensation_result_[playing_side_] = tf::Vector3(mid_point, 0.0, 0.0);


  /* Clean up */
  range_finder_data_.clear();
  error_indices_.clear();
}

// Moving average with adaptive window size. Odd window sizes only!
std::vector<double> TrajCtrl::rollingMean(std::vector<double> v, int window_size) // default window size = 29
{
  int half_window = (window_size - 1) / 2;
  int lower_boundry, upper_boundry;
  int vector_size = v.size();
  double sum;
  std::vector<double> rv;
  for (int i = 0; i < vector_size; ++i)
  {
    if (v[i] > 19.99999)
    {
      // range == 20.0 represents error, add this index to ignore list
      error_indices_.insert(i);
    }

    // Determine the boundries when the elements are not enough to fill the window
    lower_boundry = (i < half_window)? 0 : i - half_window;
    upper_boundry = ((i + half_window) > vector_size)? vector_size - 1: i + half_window;

    // Sum everything in the window
    sum = 0.0;
    for(int j = lower_boundry; j < upper_boundry; ++j)
      sum += v[j];

    // Store the average
    rv.push_back(sum / (upper_boundry - lower_boundry + 1));
  }

  return rv;
}

// Calculate difference and approximate derivative
std::vector<double> TrajCtrl::diff(std::vector<double> v)
{
  std::vector<double> rv;

  for (int i = 1; i < v.size(); ++i)
    rv.push_back(v[i] - v[i-1]);

  return rv;
}

// Find min and max of the input vector
std::pair<int, int> TrajCtrl::getPeaks(std::vector<double> v)
{
  double min{99999999.99}, max{0.0};
  int min_index, max_index;

  for (int i = 0; i < v.size(); ++i)
  {
    if (error_indices_.count(i))
      continue; // ignore data points involving 20.0 range

    if(v[i] < min)
    {
      min_index = i;
      min = v[i];
    }
    if(v[i] > max)
    {
      max_index = i;
      max = v[i];
    }
  }

  return std::pair<double,double>(min_index, max_index);
}

void TrajCtrl::debugPrintJoints(TrajCtrl::Configuration joints)
{
  #ifdef DEBUG
  for (auto q : joints) 
    ROS_INFO_STREAM(q << " " << std::flush);
  #endif
}

void TrajCtrl::debugBreak()
{
  char c;
  ROS_WARN("<<Press any key to continue>>");
  std::cin >> c;
  ros::spinOnce();
}

void TrajCtrl::debugTestFlow()
{
  int side = 0;
  int level = 9;
  int block = 0;

  playBlock(side, level, block);
}

void TrajCtrl::debugTestFunctions()
{

  moveToHomePosition(5);

  playing_side_ = 0;
  playing_level_ = 7;
  playing_block_ = 0;

  /* Move arm to probing position */
  //moveToActionPosition(PROBE, playing_side_, playing_level_, playing_block_);

  //ROS_WARN("Moved to probing position. Exectue probing is next...");
  //debugBreak();

  executeGripChangeAction();


}

void TrajCtrl::driveToEveryConfig(std::vector<TrajCtrl::Configuration> configs)
{
  /* Drive the arm to each and every possible configurations */
  for (auto config: configs)
  {
    ROS_INFO("This config:");
    debugPrintJoints(config);

    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names = UR_JOINT_NAMES_;

    trajectory_msgs::JointTrajectoryPoint current;
    current.positions = getCurrentJointState().position;
    current.time_from_start = ros::Duration(0.0);
    trajectory.points.push_back(current);

    trajectory_msgs::JointTrajectoryPoint home;
    home.positions = HOME_CONFIG_;
    home.time_from_start = ros::Duration(2.0);
    trajectory.points.push_back(home);

    trajectory_msgs::JointTrajectoryPoint next;
    next.positions = config;
    next.time_from_start = ros::Duration(4.0);
    trajectory.points.push_back(next);

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory;
    executeTrajectoryGoal(goal);
    debugBreak();
  }
}

void TrajCtrl::saveData(std::vector<std::pair<double, double>> v, std::string file_name)
{
  ROS_INFO("SAVING DATA");
  ros::spinOnce(); // Process the callback queue one last time

  if (v.empty())
  {
    ROS_WARN("Vector is empty!");
    return;
  }

  std::string output_name = "/home/clin110/data/" + file_name + "-" + std::to_string(ros::Time::now().sec) + ".txt";
  //std::ofstream out_file("/home/clin110/data/range_finder_data.txt");
  std::ofstream out_file(output_name);

  if (out_file.is_open())
  {
    ROS_INFO("File opened at %s", output_name.c_str());

    std::ostringstream first_data, second_data;
    first_data << "{\"first\": [";
    second_data << "{\"second\": [";

    for (auto it = v.begin(); it != v.end(); ++it)
    {
      first_data << it->first;
      second_data << it->second;
      if ( (it + 1) != v.end() )
      {
        first_data << ",";
        second_data << ",";
      }
    }
    first_data << "]}";
    second_data << "]}";
    out_file << "" << first_data.str() << "\n\n" << second_data.str() << "\n";
    out_file.close();

    ROS_INFO("Written %lu data points", v.size());
    // Use strcu2array(jsondecode(str)) in matlab
    //debugBreak();
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "jenga_ur5_trajectory_control");

  ros::NodeHandle nh;
  TrajCtrl trajectory_control(&nh);

  ros::spinOnce();
  //trajectory_control.debugTestFunctions();

  ROS_INFO("Initialization complete. Spinning...");
  ROS_INFO("You can run the agent now.");

  ros::spin(); // Let callbacks do the work

  return 0;
}
