/**
 * Tower Location Broadcaster
 * Infer the pose of the tower by averaging the pose of four AR markers.
 * Author: Chia-Hung Lin (clin110[AT]jhu[DOT]edu)
 * Date Created: 04/26/2018
 */
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Quaternion.h>

// Exponential moving average parameters
const double ALPHA_INITIAL = 0.9;
const double ALPHA_LOOP = 0.1;
const int LEARNING_ITERATIONS = 100;

const double MAX_P_DIFF = 0.001;
const double MAX_Q_DIFF = 0.001;

struct rpy
{
  double roll;
  double pitch;
  double yaw;
};

ros::Publisher g_vis_pub;

// Publish the jenga tower as a blue bar 75x75x270mm at the location of ar_tower_location
void publishMarkerAtTowerLocation()
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "ar_tower_location";
  marker.header.stamp = ros::Time();
  marker.ns = "jenga";
  marker.id = 6; // marker on the tool is 0, the ones on the paper are 1~4
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0.135; // The origin of the cube is at its center; move up half the height of the tower
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
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

  /* Prior knowledge of the tracking paper: AR marker is 90mm, tower is 75mm
   *  ___       ___
   * | 1 |     | 2 |
   * |___|_____|___|   ^ y
   *     |     |       |
   *     |tower|       --->x
   *  ___|_____|___
   * | 4 |     | 3 |
   * |___|     |___|
   */
  double xy_offset = 0.0925; // (90+75+20)/2 = 82.5mm
  std::array<tf::Vector3, 4> translation_to_tower;
  translation_to_tower[0] = tf::Vector3( xy_offset, -xy_offset, 0); // Marker 1
  translation_to_tower[1] = tf::Vector3(-xy_offset, -xy_offset, 0); // Marker 2
  translation_to_tower[2] = tf::Vector3(-xy_offset,  xy_offset, 0); // Marker 3
  translation_to_tower[3] = tf::Vector3( xy_offset,  xy_offset, 0); // Marker 4

  //std::array<bool, 4> frame_exists { {0, 0, 0, 0} }; // double brackets needed in C++11
  bool frame_exists = false;
  int available_frame = 0;
  std::array<std::string, 4> marker_name { {"ar_marker_1", "ar_marker_2", "ar_marker_3", "ar_marker_4"} };
  tf::StampedTransform tf_stamped;
  tf::Matrix3x3 rotation_matrix;
  std::array<tf::Vector3, 4> tf_markers_translation, prev_markers_translation;
  std::array<tf::Quaternion, 4> tf_markers_rotation, prev_markers_rotation;
  tf::Vector3 diff_translation;
  tf::Quaternion diff_rotation;
  std::array<struct rpy, 4> tf_markers_rpy, prev_markers_rpy;
  std::array<int, 4> learning_counters { {0, 0, 0, 0} }; // double brackets needed in C++11
  double alpha;
  struct rpy this_rpy;
  tf::Vector3 tower_translation;
  tf::Quaternion tower_rotation;

  ros::Rate rate(10);
  while ( nh.ok() )
  {
    tf::Vector3 sum_translation(0.0, 0.0, 0.0);
    struct rpy  sum_rpy {0.0, 0.0, 0.0};
    available_frame = 0;

    for (int i = 0; i < 4; i++)
    {
      // Store variables on the previous iteration
      prev_markers_translation[i] = tf_markers_translation[i];
      prev_markers_rotation[i] = tf_markers_rotation[i];
      prev_markers_rpy[i] = tf_markers_rpy[i];

      // Fetch marker i at current time
      ros::Time now = ros::Time::now();
      frame_exists = tf_listener.waitForTransform("base_link", marker_name[i], now, ros::Duration(0.5));

      if (!frame_exists)
      {
        continue; // Ignore unavailable marker
        learning_counters[i] = 0; // Reset moving average learning counter
      }

      // Get marker pose
      tf_listener.lookupTransform("base_link", marker_name[i], now, tf_stamped);
      tf_markers_translation[i] = tf_stamped.getOrigin();
      tf_markers_rotation[i] = tf_stamped.getRotation();

      // Check if the pose is changed
      diff_translation = prev_markers_translation[i] - tf_markers_translation[i];
      diff_rotation = prev_markers_rotation[i] - tf_markers_rotation[i];
      if (diff_translation.length() > MAX_P_DIFF || diff_rotation.length() > MAX_Q_DIFF)
      {
        // Pose drastically changed; reset counter to learn the new pose
        ROS_WARN("[marker #%d] POSE CHANGED!", i);
        ROS_WARN("[marker #%d] %f, %f", i, diff_rotation.length(), diff_translation.length());
        learning_counters[i] = 0;
      }

      // Pick appropriate alpha
      if (learning_counters[i] < LEARNING_ITERATIONS)
      {
        alpha = ALPHA_INITIAL;
        learning_counters[i]++;
      }
      else
        alpha = ALPHA_LOOP;

      // Moving average update, except on the 1st iteration
      if(learning_counters[i] > 1)
      {
        // Get RPY
        rotation_matrix.setRotation(tf_markers_rotation[i]);
        rotation_matrix.getRPY(tf_markers_rpy[i].roll, tf_markers_rpy[i].pitch, tf_markers_rpy[i].yaw);
      
        // Moving average update
        tf_markers_translation[i] = alpha * tf_markers_translation[i] + (1.0 - alpha) * prev_markers_translation[i];

        tf_markers_rpy[i].roll = alpha * tf_markers_rpy[i].roll + (1.0 - alpha) * prev_markers_rpy[i].roll;
        tf_markers_rpy[i].pitch = alpha * tf_markers_rpy[i].pitch + (1.0 - alpha) * prev_markers_rpy[i].pitch;
        tf_markers_rpy[i].yaw = alpha * tf_markers_rpy[i].yaw + (1.0 - alpha) * prev_markers_rpy[i].yaw;

        tf_markers_rotation[i].setRPY(tf_markers_rpy[i].roll, tf_markers_rpy[i].pitch, tf_markers_rpy[i].yaw);
      }

      // Update RPY
      rotation_matrix.setRotation(tf_markers_rotation[i]);
      rotation_matrix.getRPY(tf_markers_rpy[i].roll, tf_markers_rpy[i].pitch, tf_markers_rpy[i].yaw);

      // Do R*p calculation to rotate the relative tower translation based on rotation of the marker
      tf::Vector3 approx_tower_location;
      approx_tower_location.setX( rotation_matrix.getRow(0).dot(translation_to_tower[i]) );
      approx_tower_location.setY( rotation_matrix.getRow(1).dot(translation_to_tower[i]) );
      approx_tower_location.setZ( rotation_matrix.getRow(2).dot(translation_to_tower[i]) );

      // Get the approximate tower location relative to base_link by adding translation of marker: R*p + p
      approx_tower_location += tf_markers_translation[i];
      sum_translation += approx_tower_location;

      //ROS_INFO("marker translation[%d]: %.3f, %.3f, %.3f", i, tf_markers_translation[i].getX(), tf_markers_translation[i].getY(), tf_markers_translation[i].getZ());

      ROS_INFO("approx translation[%d]: %.3f, %.3f, %.3f", i, approx_tower_location.getX(), approx_tower_location.getY(), approx_tower_location.getZ());

      // Average the pose information
      sum_rpy.roll += tf_markers_rpy[i].roll;
      sum_rpy.pitch += tf_markers_rpy[i].pitch;
      sum_rpy.yaw += tf_markers_rpy[i].yaw;

      available_frame++; // Keep track of the number of available frames
    }

    if (available_frame)
    {
      // Average the translation and rpy to get the pose of the tower.
      tower_translation = sum_translation / available_frame;
      tower_translation.setZ(0.0); // The tower is ALWAYS on the table, i.e. z=0
      // Ignore all other rotations except yaw to ensure the tower is straight up
      tower_rotation.setRPY(0, 0, sum_rpy.yaw / available_frame);
    }

    tf::Transform tf_tower(tower_rotation, tower_translation);

    ROS_INFO("Available_frame: %d, x: %.3f, y: %.3f, z: %.3f\n", available_frame, tower_translation.getX(), tower_translation.getY(), tower_translation.getZ() );

    tf_broadcaster.sendTransform(
        tf::StampedTransform(tf_tower, ros::Time::now(), "base_link", "ar_tower_location") );
    publishMarkerAtTowerLocation();
  
    // Publish at 10Hz
    rate.sleep();
  }

}