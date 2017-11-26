#!/usr/bin/env python
import rospy
import math
import tf

if __name__ == '__main__':
    rospy.init_node('kinect_locator')

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(-math.pi / 2, 0, 0),
                         rospy.Time.now(),
                         "kinect2_rgb_optical_frame",
                         "map")
        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "kinect2_ir_optical_frame",
                         "kinect2_rgb_optical_frame")
        rate.sleep()
    rospy.spin()

