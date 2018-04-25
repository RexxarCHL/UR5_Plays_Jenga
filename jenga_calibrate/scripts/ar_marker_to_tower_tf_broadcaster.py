#!/usr/bin/env python
import rospy
import tf

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

		# Modify the transform for the tower location
		trans = list(trans)
		trans[2] = 0 # Set z = 0
		trans[0] -= 0.0065 # The center of the tower is (-65, -65) mm from the tag
		trans[1] -= 0.0065
		rot = (0, 0, 0, 1) # Set rotation as identity


		broadcaster.sendTransform( 
				tuple(trans),
				tuple(rot),
				rospy.Time.now(),
				'ar_tower_location',
				'base_link')
		rate.sleep()
		