#include "jenga_player/two_sides_agent.h"

// Constructor for the reflex agent
TwoSidesAgent::TwoSidesAgent(ros::NodeHandle* nh)
{
  nh_ = *nh;
  target_result_subscriber_ = nh_.subscribe("/jenga/result", 3, &TwoSidesAgent::targetResultCallback, this);
  jenga_target_publisher_ = nh_.advertise<jenga_msgs::JengaTarget>("/jenga/target", 1);

  current_tower_level_ = START_TOWER_LEVEL_;
  target_level_ = START_TARGET_LEVEL_;
  top_block_count_ = 0;

  // Wait for other nodes to subscribe to this message
  ros::Rate poll_rate(10);
  ROS_INFO("Waiting for subscribers to /jenga/target...");
  while(jenga_target_publisher_.getNumSubscribers() == 0)
  { // Will wait forever
    ROS_INFO_DELAYED_THROTTLE(10, "Still waiting...");
    ros::spinOnce();
    poll_rate.sleep();
  }

  ROS_INFO("Subscriber connected. Initialization complete.");
}

// If the result is successful, update game state. Publish the next block no matter the result.
void TwoSidesAgent::targetResultCallback(const jenga_msgs::JengaTargetResult::ConstPtr& result)
{
  previous_result_ = result->result;

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
void TwoSidesAgent::updateGameState()
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

/**
 * Get which side to drive to from tf information of the tower.
 */
int TwoSidesAgent::getSide()
{
  return level % 2; // for side 0 and 1
  // or level % 2 + 2 for side 2 and 3
}

// Try to play every legal middle block from the tower.
void TwoSidesAgent::publishNextTarget()
{
  jenga_msgs::JengaTarget target;
  target.header.stamp = ros::Time::now();
  target.side = getSide(); // Check if the tower orientation is changed drastically

  // Initialize the target message
  if (previous_result_) // If previous action is successful
  {
    // Go to next level and try to push the middle block
    target.level = previous_side_ + 2; // +2 because we want to play blocks on the same side
    target.block = 0; // Aim for the middle block
  }
  else // If previous action is NOT successful
  {
    // It is very likely that the blocks on the sides are loose
    if (previous_block_ != -1) // Previous block is not the left block 
    {
      // Stay on this level
      target.level = previous_side_;
      // If previous block is the middle block, try pushing the one to its right;
      // If previous block is the right block, try pushing the left block
      target.block = (previous_block_ == 0)? 1 : -1;
    }
    else // Previous block is the left block
    {
      // All block on this level is probed and not removeable. Move to next level
      target.level = previous_side_ + 2; // +2 because we want to play blocks on the same side
      target.block = 0;
    }
  }

  if (target.side != previous_side_ && target.side % 2 != previous_side_ % 2) 
  { 
    // Side is changed!
    ROS_WARN("SIDE CHANGED!");
    // Store the game status to play the next block on this side;
    int side_index = previous_side_ % 2;
    side_levels[side_index] = target.level;
    side_blocks[side_index] = target.block;

    // Overwrite the previous assignments to adapt to the new side
    int new_side_index = target.side;
    target.level = side_levles[new_side_index];
    target.block = side_levels[new_side_index];
  }


  ROS_INFO("Publishing target: side%d, level%d, block %d", target.side, target.level, target.block);
  jenga_target_publisher_.publish(target);

  // Increment target_level_ for next target
  target_level_++;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jenga_player");

  ros::NodeHandle nh;
  TwoSidesAgent agent(&nh);

  agent.publishNextTarget();

  ros::spin();

  return 0;
}