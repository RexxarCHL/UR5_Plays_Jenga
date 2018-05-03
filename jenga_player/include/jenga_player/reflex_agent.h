#include "jenga_player/jenga_player.h"

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