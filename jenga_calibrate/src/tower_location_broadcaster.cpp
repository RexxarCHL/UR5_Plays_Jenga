#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Quaternion.h>

struct rpy
{
  double roll;
  double pitch;
  double yall;
};

ros::Publisher g_vis_pub;

// Publish the jenga tower as a blue bar 75x75x270mm at the location of ar_tower_location
void publishMarker(tf::Vector3 translation, tf::Quaternion rotation)
{
  visualization_msgs::Marker marker;
  geometry_msgs::Quaternion q;
  tf::quaternionTFToMsg(rotation, q);

  //marker.header.frame_id = "base_link";
  marker.header.frame_id = "ar_tower_location";
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
  marker.scale.z = 0.27;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  g_vis_pub.publish( marker );
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
  std::array<std::string, 4> marker_name { {"ar_marker_1", "ar_marker_2", "ar_marker_3", "ar_marker_4"} };
  tf::StampedTransform tf_stamped;
  tf::Matrix3x3 rotation_matrix;
  std::array<tf::Vector3, 4> tf_markers_translation;
  std::array<struct rpy, 4> tf_markers_rpy;

  ros::Rate poll_rate(10);
  while ( nh.ok() )
  {
    tf::Vector3 sum_translation(0.0, 0.0, 0.0);
    struct rpy  sum_rpy {0.0, 0.0, 0.0};

    for (int i = 0; i < 4; i++)
    {
      //marker_name = "ar_marker_" + std::to_string(i + 1);
      frame_exists[i] = tf_listener.waitForTransform("base_link", marker_name[i], ros::Time(), ros::Duration(0.1));
      
      // If a frame is present, update the translation and rpy
      // If a frame is NOT present, use the previous translation and rpy
      if (frame_exists[i])
      {
        tf_listener.lookupTransform("base_link", marker_name[i], ros::Time(), tf_stamped);
        tf_markers_translation[i] = tf_stamped.getOrigin();
        rotation_matrix = tf_stamped.getBasis();
        rotation_matrix.getRPY(tf_markers_rpy[i].roll, tf_markers_rpy[i].pitch, tf_markers_rpy[i].yall);
      }

      // Sum up translation and rpy to do an averaging action.
      sum_translation += tf_markers_translation[i];
      sum_rpy.roll += tf_markers_rpy[i].roll;
      sum_rpy.pitch += tf_markers_rpy[i].pitch;
      sum_rpy.yall += tf_markers_rpy[i].yall;
    }

    // Average the translation and rpy to get the pose of the tower.
    tf::Vector3 tower_translation = sum_translation / 4.0;
    tf::Quaternion tower_rotation;
    tower_rotation.setRPY(sum_rpy.roll / 4.0, sum_rpy.pitch / 4.0, sum_rpy.yall / 4.0);
    // tower_rotation.setRPY(0, 0, sum_rpy.yall / 4.0);
    tf::Transform tf_tower(tower_rotation, tower_translation);

    tf_broadcaster.sendTransform(
        tf::StampedTransform(tf_tower, ros::Time::now(), "base_link", "ar_tower_location") );
    publishMarker(tower_translation, tower_rotation);
  }

}