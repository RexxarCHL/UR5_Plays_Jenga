#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>

bool g_track_ar_tower = true;

void trackingCallback(const std_msgs::Bool::ConstPtr& msg)
{
  g_track_ar_tower = msg->data;
  ROS_INFO("[Waypoints] Tracking = %d", g_track_ar_tower);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_waypoints");

  ros::NodeHandle nh;

  ros::Subscriber ar_tower_tracking = nh.subscribe("/jenga/tracking", 3, trackingCallback);

  tf::TransformListener tf_listener;
  tf::TransformBroadcaster tf_broadcaster;
  tf::Transform tf_direct_above;
  tf::StampedTransform ar_tower_location;
  std::vector<tf::Transform> tf_above(4), tf_side(4);

  /* tf heirachy
                    ar_tower_location (used once)
                                 |
                           roadmap_tower
                                 |
              _________roadmap_direct_above_________
             /              /        \              \
            /              /          \              \
           /              /            \              \
          /              /              \              \
          |              |              |              |
    roadmap_above0 roadmap_above1 roadmap_above2 roadmap_above3
          |              |              |              |
    roadmap_side0  roadmap_side1  roadmap_side2  roadmap_side3
   */
  /* Tower location(tf name: roadmap_tower) is assumed to be at the location
   * indicated by the ar tag
   */
  tf::Transform tf_tower; // Will initialize after ar_tower_location transform is obtained

  /* Arm position directly above the tower(tf name: roadmap_direct_above) is 
   * 15mm x 30 levels = 450mm = 0.45m higher at the tower location. 
   * Frame is rotated by pi to match the tool0 frame
   */
  int direct_above_level_offset = 30;
  nh.setParam("direct_above_level", direct_above_level_offset); // Store this as a parameter for use in trajectory control
  double z_offset = 0.015 * direct_above_level_offset - 0.0075;
  tf_direct_above.setOrigin(tf::Vector3(0.0, 0.0, z_offset));
  tf::Quaternion q;
  q.setRPY(M_PI, 0.0, 0.0);
  tf_direct_above.setRotation(q);

  /* Arm position at a higher location on each side(tf name: roadmap_above0 ~
   * roadmap_above3) is offset from roadmap_direct_above by 0.10m on the x or y
   * direction, and 0.045m on the z direction.
   * Frame is rotated in the appropriate direction to face the tool to the 
   * tower
   */
  double xy_offset = 0.12;
  int above_level_offset = 10;
  z_offset = 0.015 * above_level_offset;
  tf_above[0].setOrigin(tf::Vector3(xy_offset, 0, z_offset));
  q.setRPY(M_PI/2, 0.0, -M_PI/2);
  tf_above[0].setRotation(q);

  tf_above[1].setOrigin(tf::Vector3(0, xy_offset, z_offset));
  q.setRPY(M_PI/2, 0.0, 0.0);
  tf_above[1].setRotation(q);

  tf_above[2].setOrigin(tf::Vector3(-xy_offset, 0, z_offset));
  q.setRPY(M_PI/2, M_PI, M_PI/2);
  tf_above[2].setRotation(q);

  tf_above[3].setOrigin(tf::Vector3(0, -xy_offset, z_offset));
  q.setRPY(M_PI/2, M_PI, M_PI);
  tf_above[3].setRotation(q);

  /* Arm position at the side of the tower(tf name: roadmap_side0 ~ 
   * roadmap_side4) is at the 9th level from the table.
   */
  int side_level = 9;
  nh.setParam("side_level", side_level); // Store this as well, for use in trajectory control
  int level_difference = 
    direct_above_level_offset - above_level_offset - side_level;
  double y_offset = 0.015 * level_difference;
  tf_side[0].setOrigin(tf::Vector3(0.0, y_offset, 0.0));
  tf_side[0].setRotation(tf::Quaternion(0, 0, 0, 1));

  tf_side[1].setOrigin(tf::Vector3(0.0, y_offset, 0.0));
  tf_side[1].setRotation(tf::Quaternion(0, 0, 0, 1));

  tf_side[2].setOrigin(tf::Vector3(0.0, -y_offset, 0.0));
  tf_side[2].setRotation(tf::Quaternion(0, 0, 0, 1));

  tf_side[3].setOrigin(tf::Vector3(0.0, -y_offset, 0.0));
  tf_side[3].setRotation(tf::Quaternion(0, 0, 0, 1));

  // Wait for ar_tower_location to exist
  tf_listener.waitForTransform("base_link", "ar_tower_location", ros::Time(), ros::Duration(5.0));

  // Publish the precalculated frames
  ros::Rate rate(10.0);
  while (nh.ok()) 
  {
    ros::spinOnce();

    if (g_track_ar_tower) 
    {
      // First pass: get the tower location from the ar tag
      try
      {
        tf_listener.lookupTransform("base_link", "ar_tower_location", ros::Time(0), ar_tower_location);
      }
      catch (tf::TransformException &ex) 
      {
        ROS_ERROR("tf_waypoints: %s", ex.what());
        ROS_ERROR("tf_waypoints: will try again after 1 second...");
        ros::Duration(1.0).sleep();
        continue;
      }

      
      // Get transform from stamped transform
      tf_tower.setBasis( ar_tower_location.getBasis() );
      tf_tower.setOrigin( ar_tower_location.getOrigin() ); 
    }

    tf_broadcaster.sendTransform(
        tf::StampedTransform(tf_tower, ros::Time::now(), "base_link", "roadmap_tower") );

    tf_broadcaster.sendTransform(
        tf::StampedTransform(tf_direct_above, ros::Time::now(), "roadmap_tower", "roadmap_direct_above") );

    tf_broadcaster.sendTransform(
        tf::StampedTransform(tf_above[0], ros::Time::now(), "roadmap_direct_above", "roadmap_above0") );
    tf_broadcaster.sendTransform(
        tf::StampedTransform(tf_above[1], ros::Time::now(), "roadmap_direct_above", "roadmap_above1") );
    tf_broadcaster.sendTransform(
        tf::StampedTransform(tf_above[2], ros::Time::now(), "roadmap_direct_above", "roadmap_above2") );
    tf_broadcaster.sendTransform(
        tf::StampedTransform(tf_above[3], ros::Time::now(), "roadmap_direct_above", "roadmap_above3") );

    tf_broadcaster.sendTransform(
        tf::StampedTransform(tf_side[0], ros::Time::now(), "roadmap_above0", "roadmap_side0") );
    tf_broadcaster.sendTransform(
        tf::StampedTransform(tf_side[1], ros::Time::now(), "roadmap_above1", "roadmap_side1") );
    tf_broadcaster.sendTransform(
        tf::StampedTransform(tf_side[2], ros::Time::now(), "roadmap_above2", "roadmap_side2") );
    tf_broadcaster.sendTransform(
        tf::StampedTransform(tf_side[3], ros::Time::now(), "roadmap_above3", "roadmap_side3") );

    // Publish at 10Hz
    rate.sleep();
  }

  return 0;
}
