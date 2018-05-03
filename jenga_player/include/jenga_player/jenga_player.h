/**
 * Jenga Player
 * The AI brain that plays Jenga.
 * Author: Chia-Hung Lin (clin110[AT]jhu[DOT]edu)
 * Date Created: 04/23/2018
 */
#ifndef JENGA_PLAYER_H
#define JENGA_PLAYER_H

#include <ros/ros.h>

#include <jenga_msgs/JengaTarget.h>
//#include <jenga_msgs/JengaTargets.h>
#include <jenga_msgs/JengaTargetResult.h>

class JengaAgent // base class for all agents
{
public:
  virtual ~JengaAgent() {}; // Default destructor
  virtual void publishNextTarget() = 0; // Virtual function for the derived class to implement
protected:
  const int START_TOWER_LEVEL_ {18}; // A full tower starts at level 18
  const int START_TARGET_LEVEL_ {6}; // Start probing from level 6 due to clearance issue

  ros::NodeHandle nh_;
  ros::Publisher jenga_target_publisher_;
  ros::Subscriber target_result_subscriber_;
  
  // Game state
  int current_tower_level_;
  int top_block_count_; // Track how many blocks are currently on the top.
}; // JengaAgent

#endif // JENGA_PLAYER_H