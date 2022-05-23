#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage


def talker():
    pub = rospy.Publisher("/cam_image",CompressedImage)
    rospy.init_node('cam_to_rostopic', anonymous=True)
    rate = rospy.Rate(10)
    cam = cv2.VideoCapture(0)
        
    
    while not rospy.is_shutdown():
        ret_val, img = cam.read()
        image_GRAY=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array( cv2.imencode('.jpg', img)[1]).tostring()
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass