#include "jenga_player/jenga_player.h"

ReflexAgent::ReflexAgent(ros::NodeHandle* nh)
{
  nh_ = *nh;
  target_result_subscriber_ = nh_.subscribe("/jenga/target_result", 3, &ReflexAgent::targetResultCallback, this);
  jenga_targets_publisher_ = nh_.advertise<jenga_msgs::JengaTargets>("/jenga/targets", 1, true);
}

void ReflexAgent::targetResultCallback(const jenga_msgs::JengaTargetResult::ConstPtr& result)
{
  // do something
}

void ReflexAgent::publishNextTarget()
{
  // TODO
  ROS_INFO("%d %d", START_TOWER_LEVEL_, START_TARGET_LEVEL_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jenga_player");

  ros::NodeHandle nh;
  ReflexAgent agent();

  ros::spin();

  return 0;
}