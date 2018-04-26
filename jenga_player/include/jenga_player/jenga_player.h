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
  int target_level_;
}; // JengaAgent

/**
 * ReflexAgent: 
 *   A simple reflex agent that tries to play every legal middle block from the tower.
 */
class ReflexAgent : public JengaAgent
{
public:
  ReflexAgent(ros::NodeHandle* nh);
  void publishNextTarget(); // Inherited
private:
  int top_block_count_; // Track how many blocks are currently on the top.
  /**
   * Process the result and publish the next target block.
   */
  void targetResultCallback(const jenga_msgs::JengaTargetResult::ConstPtr& result);
  /**
   * Increment top block count; increment current tower level if 3 blocks are on the top.
   */
  void updateGameState();
  /**
   * Get which side to drive to from the input level.
   */
  int getSide(int level);

}; // ReflexAgent

#endif // JENGA_PLAYER_H