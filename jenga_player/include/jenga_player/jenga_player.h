/**
 * Jenga Player
 * The AI brain that plays Jenga. A simple reflex agent that tries to play every middle block from level 6 and up.
 * Author: Chia-Hung Lin (clin110[AT]jhu[DOT]edu)
 * Date Created: 04/23/2018
 */
#ifndef JENGA_PLAYER_H
#define JENGA_PLAYER_H

#include <ros/ros.h>

#include <jenga_msgs/JengaTarget.h>
#include <jenga_msgs/JengaTargets.h>
#include <jenga_msgs/JengaTargetResult.h>

class JengaAgent // base class for all agents
{
public:
  virtual ~JengaAgent() {};
  virtual void publishNextTarget() = 0;
protected:
  const int START_TOWER_LEVEL_ {18};
  const int START_TARGET_LEVEL_ {6}; 
  ros::NodeHandle nh_;
  ros::Publisher jenga_targets_publisher_;
  ros::Subscriber target_result_subscriber_;
  int current_tower_level_;
  int target_level_;
}; // JengaAgent

class ReflexAgent : public JengaAgent
{
public:
  ReflexAgent(ros::NodeHandle* nh);
  void publishNextTarget();
private:
  void targetResultCallback(const jenga_msgs::JengaTargetResult::ConstPtr& result);
}; // ReflexAgent

#endif // JENGA_PLAYER_H