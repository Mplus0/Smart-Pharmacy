#!/usr/bin/env python
# encoding: utf-8

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

def Cam_RGB_Callback(msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg,"bgr8")

    except CvBridgeError as e:
        rospy.logerr("格式转换错误：%s",e)
        return
    
    cv2.imshow("RGB",cv_image)
    cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("demo_cv_image")
    rgb_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,Cam_RGB_Callback,queue_size=10)
    rospy.spin()