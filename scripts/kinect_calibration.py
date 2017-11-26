#!/usr/bin/env python
import rospy
import math
import tf
import numpy as np
from helpers import find_optimal_transform

if __name__ == '__main__':
    rospy.init_node('kinect_locator')

    root_frame = "base_link" # should this be world? map?
    kinect_frame = "kinect2_rgb_optical_frame"
    sawyer_frame_prefix = "head_marker_"
    kinect_frame_prefix = "kinect_marker_"

    br = tf.TransformBroadcaster()
    tl = tf.TransformListener()

    rospy.sleep(3.0) # wait for some tag data to come ink

    def get_tag_ids(frames, prefix):
        frames = filter(lambda f: f.startswith(prefix), frames)
        return set([int(f[len(prefix):]) for f in frames])

    t = None
    q = None
    error = float('inf')
    while error > 0.02:
        frames = tl.getFrameStrings()

        kinect_tags = get_tag_ids(frames, kinect_frame_prefix)
        sawyer_tags = get_tag_ids(frames, sawyer_frame_prefix)
        mutual_tags = kinect_tags & sawyer_tags

        if len(mutual_tags) == 18:

            sawyer_markers = []
            kinect_markers = []
            for tag_id in mutual_tags:

                tag_frame = kinect_frame_prefix + str(tag_id)
                pos, _ = tl.lookupTransform(tag_frame, kinect_frame, rospy.Time(0))
                kinect_markers.append(np.array(pos))

                tag_frame = sawyer_frame_prefix + str(tag_id)
                pos, _ = tl.lookupTransform(tag_frame, root_frame, rospy.Time(0))
                sawyer_markers.append(np.array(pos))

            kinect_markers = np.vstack(kinect_markers)
            sawyer_markers = np.vstack(sawyer_markers)

            t, q, error = find_optimal_transform(kinect_markers, sawyer_markers)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        br.sendTransform(t,
                         q,
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

