#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
	rospy.init_node('ar_marker_to_tower_tf_broadcaster')

	listener = tf.TransformListener()
	broadcaster = tf.TransformBroadcaster()

	frame_exists = \
		listener.waitForTransform('base_link', 'ar_marker_1', 
								  rospy.Time(0),rospy.Duration(5.0))


	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		broadcaster.sendTransform( 
				(-0.065, -0.065, 0),
				(0, 0, 0, 1),
				rospy.Time.now(),
				'ar_tower_location',
				'ar_marker_1')
		rate.sleep()
		