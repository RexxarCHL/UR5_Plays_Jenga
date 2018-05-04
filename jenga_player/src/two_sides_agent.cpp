#include "jenga_player/two_sides_agent.h"

// Constructor for the reflex agent
TwoSidesAgent::TwoSidesAgent(ros::NodeHandle* nh)
{
  nh_ = *nh;
  target_result_subscriber_ = nh_.subscribe("/jenga/result", 3, &TwoSidesAgent::targetResultCallback, this);
  jenga_target_publisher_ = nh_.advertise<jenga_msgs::JengaTarget>("/jenga/target", 1);

  current_tower_level_ = START_TOWER_LEVEL_;

  top_block_count_ = 0;
  playing_side_ = getSide();
  side_levels_[0] = START_TARGET_LEVEL_ + 1;
  side_levels_[1] = START_TARGET_LEVEL_;
  side_blocks_[0] = 0;
  side_blocks_[1] = 0;
  playing_level_ = side_levels_[playing_side_ % 2] - 2;
  playing_block_ = -1;
  playing_result_ = false;

  // Wait for other nodes to subscribe to this message
  ros::Rate poll_rate(10);
  ROS_INFO("Waiting for subscribers to /jenga/target...");
  while(jenga_target_publisher_.getNumSubscribers() == 0)
  { // Will wait forever
    ROS_INFO_DELAYED_THROTTLE(5, "Still waiting...");
    ros::spinOnce();
    poll_rate.sleep();
  }

  ROS_INFO("Subscriber connected. Initialization complete.");
}

// If the result is successful, update game state. Publish the next block no matter the result.
void TwoSidesAgent::targetResultCallback(const jenga_msgs::JengaTargetResult::ConstPtr& result)
{
  playing_result_ = result->result;

  // do something
  if (result->result) // Probing successful
  {
    updateGameState();
  }

  if (side_levels_[playing_side_ % 2] < current_tower_level_ - 2)
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
      current_tower_level_, side_levels_[playing_side_], top_block_count_);
}

/**
 * Get which side to drive to from tf information of the tower.
 */
int TwoSidesAgent::getSide()
{
  std::array<tf::Transform, 4> tf_side;
  std::array<double, 4> angles, distances;
  double min_distance = 999999.99;
  double delta = 0.001;
  int min_index = -1;
  for (int i = 0; i < 4; ++i)
  {
    tf_side[i] = retrieveTransform(SIDE_FRAME_NAMES[i]);
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

  ROS_INFO("Returning side: %d", (min_index + 1) % 2);
  // Return the side that is NOT the min distance side
  return (min_index + 1) % 2;
}

// Try to play every legal middle block from the tower.
void TwoSidesAgent::publishNextTarget()
{
  jenga_msgs::JengaTarget target;
  target.header.stamp = ros::Time::now();
  target.side = getSide(); // Check if the tower orientation is changed drastically

  // Robot will try to play block 1 -> 0 -> -1

  // If previous action is successful AND middle or right block is extracted
  if (playing_result_ && playing_block_ < 1)
  {
    // Go to next level and try to push the middle block
    target.level = playing_level_ + 2; // +2 because we want to play blocks on the same side
    target.block = 0; // Aim for the middle block
  }
  else // If previous action is NOT successful OR  action is successful and a side block is played
  {
    // It is very likely that the blocks on the sides are loose
    if (playing_block_ != -1) // Previous block is not the left block 
    {
      // Stay on this level
      target.level = playing_level_;
      // If previous block is the middle block, try pushing the one to its right;
      // If previous block is the right block, try pushing the left block
      target.block = (playing_block_ == 0)? 1 : -1;
    }
    else // Previous block is the left block
    {
      // All block on this level is probed and not removeable. Move to next level
      target.level = playing_level_ + 2; // +2 because we want to play blocks on the same side
      target.block = 0;
    }
  }

  if (target.side != playing_side_ && target.side % 2 != playing_side_ % 2) 
  { 
    // Side is changed!
    ROS_WARN("SIDE CHANGED!");
    // Store the game status to play the next block on this side;
    side_levels_[playing_side_ % 2] = target.level;
    side_blocks_[playing_side_ % 2] = target.block;

    // Overwrite the previous assignments to adapt to the new side
    playing_side_ = target.side;
    target.level = side_levels_[playing_side_ % 2];
    target.block = side_blocks_[playing_side_ % 2];
  }

  ROS_INFO("Publishing target: side%d, level%d, block %d", target.side, target.level, target.block);
  jenga_target_publisher_.publish(target);

  playing_level_ = target.level;
  playing_block_ = target.block;
} 

/**
 * Wait and return the transfrom by the input name
 */
tf::Transform TwoSidesAgent::retrieveTransform(std::string frame_name)
{
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jenga_player");

  ros::NodeHandle nh;
  TwoSidesAgent agent(&nh);

  agent.publishNextTarget();

  ros::spin();

  return 0;
}