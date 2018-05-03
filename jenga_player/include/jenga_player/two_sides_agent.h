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
  int playing_side_; // Store which side to push from 
  std::array<int, 2> side_levels_;
  std::array<int, 2> side_blocks_;
  int playing_level_;
  int playing_block_;
  bool playing_result_;
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