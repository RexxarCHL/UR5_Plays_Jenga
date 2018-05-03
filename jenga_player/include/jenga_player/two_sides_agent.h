#include "jenga_player/jenga_player.h"

#include <tf/transform_listener.h>

/**
 * TwoSidesAgent: 
 *   An agent that tries to play every block on two easily accessible sides
 */
class TwoSidesAgent : public JengaAgent
{
public:
  TwoSidesAgent(ros::NodeHandle* nh);
  void publishNextTarget(); // Inherited
private:
  tf::TransformListener tf_listener_;
  int top_block_count_; // Track how many blocks are currently on the top.
  int playing_side_; // Store which side to push from 
  int* playing_side_level;
  int previous_side_;
  int previous_level_;
  int previous_block_;
  bool previous_result_;
  std::array<int, 2> side_levels;
  /**
   * Process the result and publish the next target block.
   */
  void targetResultCallback(const jenga_msgs::JengaTargetResult::ConstPtr& result);
  /**
   * Increment top block count; increment current tower level if 3 blocks are on the top.
   */
  void updateGameState();
  /**
   * Get which side to drive to from tf information of the tower.
   */
  int getSide();

}; // TwoSidesAgent