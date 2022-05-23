#!/usr/bin/env python

import sys, time
import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

class Face_n_Eye_detection:

    def __init__(self):
        self.subscriber = rospy.Subscriber("/cam_image",CompressedImage, self.callback,  queue_size = 1)
        self.publisher = rospy.Publisher("/cmd_vel",Twist)
       
    def callback(self, ros_data):        
        msg = Twist()
        msg.linear.x = 0 # stopping 

        cascPath = "/home/luqman/catkin_ws/src/rpi_car/haarcascades/haarcascade_frontalface_default.xml"
        faceCascade = cv2.CascadeClassifier(cascPath)

        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        
        faces = faceCascade.detectMultiScale(image_np, 1.3, 5)
        self.publisher.publish(msg)
        
        for (x, y, w, h) in faces:
            cv2.rectangle(image_np, (x, y), (x+w, y+h), (0, 255, 0), 2)
            print "Moving the Car"
            msg.linear.x = 1 # move the robot 
            self.publisher.publish(msg)
            
        cv2.imshow('Video', image_np)
        cv2.waitKey(2)

def main(args):
    ic = Face_n_Eye_detection()
    rospy.init_node('Face_detector_NODE', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)