#!/usr/bin/env python

import roslib
import rospy
import cv2
import process as ps

from std_msgs.msg import String
from std_msgs.msg import UInt16MultiArray
from drv_msgs.msg import recognized_objects
from drv_msgs.srv import *
from cv_bridge import CvBridge, CvBridgeError

roslib.load_manifest('drv_recognize')


def handle_recognize(req):
    rsp = recognizeResponse()

    bridge = CvBridge()
    try:
        # Convert rosmsg to cv image
        # np_array = np.fromstring(req.img_in.data, np.uint8)
        # image = cv2.imdecode(np_array, cv2.CV_LOAD_IMAGE_COLOR)

        cv_image = bridge.imgmsg_to_cv2(req.img_in, "bgr8")
        (rows, cols, channels) = cv_image.shape

        if cols != 640 or rows != 480:
            rospy.WARN('Can not get image.')
            return rsp
        else:
            result = ps.process(cv_im