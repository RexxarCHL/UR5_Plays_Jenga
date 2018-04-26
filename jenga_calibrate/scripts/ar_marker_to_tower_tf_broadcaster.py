#!/usr/bin/env python
import rospy
import tf
import numpy as np

if __name__ == '__main__':
	rospy.init_node('base_link_to_camera_tf_broadcaster')

	listener = tf.TransformListener()
	broadcaster = tf.TransformBroadcaster()

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try: # Get the transformation to ar_marker_1 from base_link
			(trans, rot) = listener.lookupTransform('base_link', 'ar_marker_1', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		tf_base_marker = listener.fromTranslationRotation(trans, rot)

		rpy = tf.transformations.euler_from_quaternion(rot)
		rot = tf.transformations.quaternion_from_euler(-rpy[0], -rpy[1], 0)
		tf_marker_to_tower = listener.fromTranslationRotation(
				(-0.065, -0.065, 0),
				tuple(rot) )

		tf_base_tower = tf_base_marker * tf_marker_to_tower
		_, _, rpy, trans, _ = tf.transformations.decompose_matrix(tf_base_tower)
		rot = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])

		broadcaster.sendTransform( 
				tuple(trans),
				tuple(rot),
				rospy.Time.now(),
				'ar_tower_location',
				'base_link')
		rate.sleep()
		