#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import pytesseract
from PIL import Image
from turtlesim.msg import Pose

def talker():
    pub = rospy.Publisher("/robot_number",Pose)
    rospy.init_node('Robot_Friend_Finder', anonymous=True)
    rate = rospy.Rate(10)
    robot_friend=Pose()  
    result= pytesseract.image_to_string(Image.open('/home/luqman/catkin_ws/src/rpi_car/scripts/3.jpg')
                                        ,config='-psm 10')
    result = float(result)

    if result == 1:
        robot_friend.x=1
        robot_friend.y=2

    elif result == 2:
        robot_friend.x=7
        robot_friend.y=7

    elif result == 3:
        robot_friend.x=4
        robot_friend.y=4
            

    pub.publish(robot_friend)

    print(result)
        
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass