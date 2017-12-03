#!/usr/bin/env python

# Modified from intera's camera_display.py example

import argparse
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
import intera_interface

def show_image_callback(img_data, (edge_detection, window_name)):
    """The callback function to show image by using CvBridge and cv
    """
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
    except CvBridgeError, err:
        rospy.logerr(err)
        return
    if edge_detection == True:
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        # customize the second and the third argument, minVal and maxVal
        # in function cv2.Canny if needed
        get_edge = cv2.Canny(blurred, 10, 100)
        cv_image = np.hstack([get_edge])
    edge_str = "(Edge Detection)" if edge_detection else ''
    cv_win_name = ' '.join([window_name, edge_str])
    cv2.namedWindow(cv_win_name, 0)
    # refresh the image on the screen
    cv2.imshow(cv_win_name, cv_image)
    cv2.waitKey(3)

def main():
    """Camera Display Example
    """
    rp = intera_interface.RobotParams()
    valid_cameras = rp.get_camera_names()
    if not valid_cameras:
        rp.log_message(("Cannot detect any camera_config"
            " parameters on this robot. Exiting."), "ERROR")
        return
    arg_fmt = argparse.RawDescriptionHelpFormatter

    print("Initializing node... ")
    rospy.init_node('camera_display', anonymous=True)
    camera = intera_interface.Cameras()
    if not camera.verify_camera_exists("right_hand_camera"):
        rospy.logerr("Invalid camera name, exiting the example.")
        return
    camera.start_streaming("right_hand_camera")
    rectify_image = True
    use_canny_edge = False
    camera.set_callback("right_hand_camera", show_image_callback,
        rectify_image=rectify_image, callback_args=(use_canny_edge, "right_hand_camera"))

    def clean_shutdown():
        print("Shutting down camera_display node.")
        cv2.destroyAllWindows()

    rospy.on_shutdown(clean_shutdown)
    rospy.loginfo("Camera_display node running. Ctrl-c to quit")
    rospy.spin()


if __name__ == '__main__':
    main()
