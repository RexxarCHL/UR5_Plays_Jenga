#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Quaternion.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

ros::Publisher g_vis_pub;

void publishMarker(tf::Vector3 translation, tf::Quaternion rotation)
{
  visualization_msgs::Marker marker;
  geometry_msgs::Quaternion q;
  tf::quaternionTFToMsg(rotation, q);

  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();
  marker.ns = "basic_shapes";
  marker.id = 5; // marker on the tool is 0, the ones on the paper are 1~4
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = translation[0];
  marker.pose.position.y = translation[1];
  marker.pose.position.z = translation[2] + 0.225;
  marker.pose.orientation = q;
  marker.scale.x = 0.075;
  marker.scale.y = 0.075;
  marker.scale.z = 0.45;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  g_vis_pub.publish( marker );
}

void debugBreak()
{
  char c;
  std::cin >> c;
  ros::spinOnce();
  //ros::Duration(3.0).sleep();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ar_marker_to_tower_tf_broadcaster");

  ros::NodeHandle nh;
  g_vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  tf::TransformListener tf_listener;
  tf::TransformBroadcaster tf_broadcaster;

  // Wait for a wild ar_marker_1 to appear
  tf_listener.waitForTransform("ar_marker_1", "base_link", ros::Time(), ros::Duration(5.0));

  std::array<bool, 4> frame_exists { {0, 0, 0, 0} }; // double brackets needed in C++11
  std::string marker_name;
  tf::StampedTransform tf_stamped;
  std::array<tf::Vector3, 4> tf_markers_translation;
  std::array<tf::Quaternion, 4> tf_markers_quaternion;

  ros::Rate poll_rate(10);
  while ( nh.ok() )
  {
    tf::Vector3 sum_translation(0, 0, 0);
    tf::Quaternion sum_quaternion = tf::Quaternion::getIdentity();

    for (int i = 0; i < 4; i++)
    {
      marker_name = "ar_marker_" + std::to_string(i + 1);
      frame_exists[i] = tf_listener.waitForTransform("base_link", marker_name, ros::Time(), ros::Duration(0.1));
      
      if (frame_exists[i])
      {
        tf_listener.lookupTransform("base_link", marker_name, ros::Time(), tf_stamped);
        tf_markers_translation[i] = tf_stamped.getOrigin();
        tf_markers_quaternion[i] = tf_stamped.getRotation();
        //tf_markers[i] = tf::Transform( tf_markers_quaternion[i], tf_markers_translation[i] );
      }

      sum_translation += tf_markers_translation[i];
      sum_quaternion += tf_markers_quaternion[i];
    }

    tf::Vector3 tower_translation = sum_translation / 4.0;
    tf::Quaternion tower_rotation = (sum_quaternion / 4.0).normalized();
    tf::Transform tf_tower(tower_rotation, tower_translation);

    Eigen::Vector2d v;

    tf_broadcaster.sendTransform(
        tf::StampedTransform(tf_tower, ros::Time::now(), "base_link", "ar_tower_location") );
    publishMarker(tower_translation, tower_rotation);
  }

}