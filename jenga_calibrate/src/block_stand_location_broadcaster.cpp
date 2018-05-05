/**
 * Block Stand Location Broadcaster
 * Publish the pose of the block stand from the pose of ar_marker_5
 * Author: Chia-Hung Lin (clin110[AT]jhu[DOT]edu)
 * Date Created: 05/03/2018
 */
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using namespace std;

// Exponential moving average parameters
const double ALPHA_INITIAL = 0.9;
const double ALPHA_LOOP = 0.1;
const int LEARNING_ITERATIONS = 100;

const double MAX_P_DIFF = 0.001;
const double MAX_Q_DIFF = 0.001;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "block_stand_tf_broadcaster");

	ros::NodeHandle nh;
	tf::TransformListener tf_listener;
	tf::TransformBroadcaster tf_broadcaster;

  /* Initialize relative transforms */
  // ar_marker_5 to roadmap/block_rest
  tf::Transform tf_marker_to_rest(
      tf::Quaternion(0.962162894354454, 0.044047849016084, 0.115541415343399, 0.242801427230072), 
      tf::Vector3(0.093663639678848, 0.084229382378222, 0.037221290122700) );

  // roadmap/block_rest to roadmap_blockTwoSidesAgent
  tf::Quaternion q; q.setRPY(-M_PI/2-M_PI/20, -M_PI/10, M_PI);
  tf::Transform tf_rest_to_drop(q, tf::Vector3(0.05, 0.05, 0.03));

  // roadmap/block_rest to roadmap_block_pickup
  q; q.setRPY(0, M_PI, -M_PI/2);
  tf::Transform tf_rest_to_pickup(q, tf::Vector3(0.0, 0.0, -0.07));


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

  tf::Vector3 t(0, 0, 0), prev_t, diff_t; // current, previous, and difference of translation of the marker
  q.setRPY(0, 0, 0); // current rotation of the marker (reuse)
  tf::Quaternion prev_q, diff_q; // previous and difference of rotation of the marker
  tf::Matrix3x3 rot_mtx; // rotation matrix, for extracting rpy
  double r{0.0}, prev_r;
  double p{0.0}, prev_p;
  double y{0.0}, prev_y;
  int learning_counter = 0; // Used to switch from alpha_initial to alpha_loop
  double alpha = ALPHA_INITIAL;

  ros::Rate rate(10);
  while( nh.ok() )
  {
    // Store variables on the previous iteration
    prev_t = t;
    prev_q = q;
    prev_r = r;
    prev_p = p;
    prev_y = y;

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
    q = tf_stamped.getRotation();
    t = tf_stamped.getOrigin();

    // Check if pose is changed
    diff_q = prev_q - q;
    diff_t = prev_t - t;
    if (diff_q.length() > MAX_Q_DIFF || diff_t.length() > MAX_P_DIFF)
    {
      // Pose drastically changed; reset counter to learn the new positions
      ROS_WARN("[block stand] POSE CHANGED!");
      ROS_WARN("[block stand] %f, %f", diff_q.length(), diff_t.length());
      learning_counter = 0; // reset counter to use alpha_initial
    }
    ROS_WARN("diff: %f, %f", diff_q.length(), diff_t.length());

    if (learning_counter < LEARNING_ITERATIONS)
    {
      alpha = ALPHA_INITIAL;
      learning_counter++;
    }
    else
    {
      ROS_INFO("loop");
      alpha = ALPHA_LOOP;
    }

    rot_mtx.setRotation(q);
    rot_mtx.getRPY(r, p, y);
    if(learning_counter > 1) // Update except on the 1st iteration
    {
      ROS_INFO("current : x: %f, y:%f, z: %f, r:%f, p:%f, y:%f", t.getX(), t.getY(), t.getZ(), r, p, y);
      ROS_INFO("previous: x: %f, y:%f, z: %f, r:%f, p:%f, y:%f", prev_t.getX(), prev_t.getY(), prev_t.getZ(), prev_r, prev_p, prev_y);

      t = alpha * t + (1.0 - alpha) * prev_t;
      r = alpha * r + (1.0 - alpha) * prev_r;
      p = alpha * p + (1.0 - alpha) * prev_p;
      y = alpha * y + (1.0 - alpha) * prev_y;

      q.setRPY(r, p, y);
    }

     ROS_INFO("after : x: %f, y:%f, z: %f, r:%f, p:%f, y:%f", t.getX(), t.getY(), t.getZ(), r, p, y);

    // Construct the transforms
    tf::Transform tf_marker(q, t);
    tf::Transform tf_rest = tf_marker * tf_marker_to_rest;
    tf::Transform tf_drop = tf_rest * tf_rest_to_drop;
    tf::Transform tf_pickup = tf_rest * tf_rest_to_pickup;
    
    // Send the transforms
    tf_broadcaster.sendTransform(
        tf::StampedTransform(tf_rest, ros::Time::now(), "base_link", "roadmap_block_rest") );
    tf_broadcaster.sendTransform(
        tf::StampedTransform(tf_drop, ros::Time::now(), "base_link", "roadmap_block_drop") );
    tf_broadcaster.sendTransform(
        tf::StampedTransform(tf_pickup, ros::Time::now(), "base_link", "roadmap_block_pickup") );
  
    // Publish at 10Hz
    rate.sleep();
  }
}