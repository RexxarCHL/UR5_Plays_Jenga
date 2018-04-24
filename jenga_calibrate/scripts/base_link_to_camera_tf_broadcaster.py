#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
	rospy.init_node('base_link_to_camera_tf_broadcaster')
	transform = rospy.get_param('/camera/transformation')
	br = tf.TransformBroadcaster()

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		br.sendTransform( 
				tuple(transform[0:3]),
				tuple(transform[3:7]),
				rospy.Time.now(),
				'camera',
				'base_link')
		rate.sleep()
		