/**
 * Block Stand Location Broadcaster
 * Publish the pose of the block stand from the pose of ar_marker_5
 * Author: Chia-Hung Lin (clin110[AT]jhu[DOT]edu)
 * Date Created: 05/03/2018
 */
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "block_stand_tf_broadcaster");

	ros::NodeHandle nh;
	tf::TransformListener tf_listener;
	tf::TransformBroadcaster tf_broadcaster;

  /* Initialize relative transforms */
  // ar_marker_5 to roadmap/block_rest
  tf::Matrix3x3 rot(
     0.000000, 0.965926, -0.258819, 
    -0.866025, 0.129410,  0.482963,
     0.500000, 0.224144,  0.836516); // Data acquired from CAD 
  tf::Transform tf_marker_to_rest(rot, tf::Vector3(0.0929930, 0.0839667, 0.0345500));

  // roadmap/block_rest to roadmap_block_place
  tf::Transform tf_rest_to_place(tf::Quaternion::getIdentity(), tf::Vector3(0.03, 0.03, 0.01));

  // roadmap/block_rest to roadmap_block_pickup
  //tf::Transform tf_pickup(tf::Quaternion::getIdentity(), tf::Vector3(0.0, 0.0, 0.01));


	/* Wait for ar_marker_5 to spawn */
  bool frame_exists = tf_listener.waitForTransform("base_link", "ar_marker_5", ros::Time(), ros::Duration(1.0));

  tf::StampedTransform tf_stamped;
  while(!frame_exists)
  {
    ROS_WARN("Frame \"ar_marker_5\" does not exist; retrying...");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    frame_exists = tf_listener.waitForTransform("base_link", "ar_marker_5", ros::Time(), ros::Duration(1.0));
  }

  ros::Rate rate(100);
  while( nh.ok() )
  {
    ros::Time now = ros::Time::now();
    frame_exists = tf_listener.waitForTransform("base_link", "ar_marker_5", now, ros::Duration(0.5));

    if(!frame_exists) // Wait for marker to exist
    {
      ROS_WARN("Frame \"ar_marker_5\" does not exist at time %u; retrying...", now.sec);
      ros::spinOnce();
      ros::Duration(0.5).sleep();
      continue;
    }

    // Get the marker pose
    tf_listener.lookupTransform("base_link", "ar_marker_5", now, tf_stamped);

    // Construct the transforms
    tf::Transform tf_marker( tf_stamped.getRotation(), tf_stamped.getOrigin() );
    tf::Transform tf_rest = tf_marker * tf_marker_to_rest;
    tf::Transform tf_place = tf_rest * tf_rest_to_place;
    
    // Send the transforms
    tf_broadcaster.sendTransform(
        tf::StampedTransform(tf_rest, ros::Time::now(), "base_link", "roadmap_block_rest") );
    tf_broadcaster.sendTransform(
        tf::StampedTransform(tf_place, ros::Time::now(), "base_link", "roadmap_block_place") );
    //tf_broadcaster.sendTransform(
    //    tf::StampedTransform(tf_pickup, ros::Time::now(), "base_link", "roadmap/block_pickup") );
  
    // Publish at 100Hz
    rate.sleep();
  }
}