#include "jenga_player/jenga_player.h"

// Constructor for the reflex agent
ReflexAgent::ReflexAgent(ros::NodeHandle* nh)
{
  ROS_INFO("1");
  nh_ = *nh;
  target_result_subscriber_ = nh_.subscribe("/jenga/target_result", 3, &ReflexAgent::targetResultCallback, this);
  jenga_target_publisher_ = nh_.advertise<jenga_msgs::JengaTarget>("/jenga/target", 1);

  ROS_INFO("2");

  current_tower_level_ = START_TOWER_LEVEL_;
  target_level_ = START_TARGET_LEVEL_;
  top_block_count_ = 0;

  publishNextTarget(); // Publish the first block immediately after initialization
}

// If the result is successful, update game state. Publish the next block no matter the result.
void ReflexAgent::targetResultCallback(const jenga_msgs::JengaTargetResult::ConstPtr& result)
{
  // do something
  if (result->result) // Probing successful
  {
    updateGameState();
  }

  if (target_level_ < current_tower_level_ - 1)
    publishNextTarget(); // Next move is legal. OK to publish
  else
  {
    // No more possible moves
    ROS_FATAL("No more possible moves. Spinning...");
    ros::spin();
  }
}

// Increment top block count; increment current tower level if 3 blocks are on the top.
void ReflexAgent::updateGameState()
{
  // Put another block on top of the tower
  top_block_count_ += 1;

  // One full level of tower completed
  if (top_block_count_ > 3)
  {
    current_tower_level_ += 1;
    top_block_count_ = 0;
  }

  ROS_INFO("Current tower level: %d, target level: %d, block count: %d", 
      current_tower_level_, target_level_, top_block_count_);
}

// Get which side to drive to from the input level.
int ReflexAgent::getSide(int level)
{
  return level % 2; // for side 0 and 1
  // or level % 2 + 2 for side 2 and 3
}

// Try to play every legal middle block from the tower.
void ReflexAgent::publishNextTarget()
{
  jenga_msgs::JengaTarget target;

  // Initialize the target message
  target.side = getSide(target_level_);
  target.level = target_level_;
  target.block = 0; // Aim for the middle block

  // Increment target_level_ for next target
  target_level_++;


  ROS_INFO("Publishing target: side%d, level%d", target.side, target.level);
  jenga_target_publisher_.publish(target);
  ROS_INFO("OK");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jenga_player");

  ros::NodeHandle nh;
  ReflexAgent agent(&nh);


  ros::spin();

  return 0;
}