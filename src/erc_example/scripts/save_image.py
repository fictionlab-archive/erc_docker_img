#! /usr/bin/env python

import sys
from datetime import datetime

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2

rospy.init_node('save_image_node', anonymous=True)

image_topic = "/camera/image_raw"
if len(sys.argv) > 1:
    image_topic = sys.argv[1]

bridge = CvBridge()

try:
    image = rospy.wait_for_message(image_topic, Image, timeout=5.0)
except rospy.ROSException as e:
    rospy.logerr("Failed to retrieve image: %s" % (e,))
    sys.exit(1)

try:
    cv2_img = bridge.imgmsg_to_cv2(image, "bgr8")
except CvBridgeError as e:
    rospy.logerr("Failed to convert image: %s" % (e,))
    sys.exit(2)

img_time = datetime.fromtimestamp(image.header.stamp.secs)
time_str = img_time.strftime("%Y-%m-%d-%H-%M-%S")

cv2.imwrite('/image_%s.png' % (time_str,), cv2_img)