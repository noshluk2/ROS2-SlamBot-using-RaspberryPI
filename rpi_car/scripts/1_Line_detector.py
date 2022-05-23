#!/usr/bin/env python

import sys
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage

# cv_bridge it does not support CompressedImage in python

class Lines_detection:

    def __init__(self):
        
        self.subscriber = rospy.Subscriber("luqman_bot/camera1/image_raw/compressed",
                                            CompressedImage, self.callback,  queue_size = 1)


    def callback(self, ros_data):
     # You are here when ever you recieve data    
        
        # Image decoding according to Opencv requirment
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        #processing the image data
        image_GRAY=cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY) # grayscaling
        lsd = cv2.createLineSegmentDetector(0) 
        lines = lsd.detect(image_GRAY)[0]  # finding lines on the image
        drawn_img = lsd.drawSegments(image_np,lines) # drawing lines on the input image
        # Visualizing the final output
        cv2.imshow("Orginal with line", drawn_img)
        key = cv2.waitKey(2)


def main():
    ic = Lines_detection()
    rospy.init_node('Lines detection', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print " Time to Kill"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()