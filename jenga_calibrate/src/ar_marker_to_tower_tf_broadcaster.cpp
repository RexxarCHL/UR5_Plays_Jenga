#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ar_marker_to_tower_tf_broadcaster");

  ros::NodeHandle nh;

  tf::TransformListener tf_listener;
  tf::TransformBroadcaster tf_broadcaster;

  /* Look up transfrom from base_link to ar_marker_1 */
  bool frame_exists = tf_listener.waitForTransform("ar_marker_1", "base_link", ros::Time(), ros::Duration(1.0));
  while (!frame_exists)
  {
    ROS_WARN("No transformation ar_marker_1 relative to base_link; retrying...");
    ros::spinOnce();
    ros::Duration(1.0).sleep()
    bool frame_exists = tf_listener.waitForTransform("ar_marker_1", "base_link", ros::Time(), ros::Duration(1.0));
  }
  tf::StampedTransform tf_stamped
  tf_listener.lookupTransform("ar_marker_1", "base_link", ros::Time(0), tf_stamped);

  /* Construct transform from base_link to ar_marker_1 */
  tf::Transform tf_base_marker( tf_stamped.getRotation(), tf_stamped.getOrigin() );

  /* Construct transfrom from ar_marker_1 to ar_tower_location */
  // Rotation
  tf::Matrix3x3 rot_base_marker = tf_base_marker.getBasis();
  tf::tfScalar r, p, y;
  rot_base_marker.getRPY(&r, &p, &y);
  tf::Quaternion q_marker_tower;
  q_marker_tower.setRPY(-r, -p, 0);
  // Translation
  tf::Vector3 translation_marker_tower(-0.065, -0.065, 0);
  // Construct transform 
  tf::Transform tf_marker_tower(q_marker_tower, translation_marker_tower);

  /* Calculate the pose of tower based on tf_base_marker and tf_marker_tower */
  tf::Transform tf_base_tower = tf_base_marker * tf_marker_tower;


}