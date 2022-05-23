#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt


class TurtleBot:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.v_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10) # velocity publisher
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, self.update_pose)
        self.goal_subscriber = rospy.Subscriber('/robot_number',Pose, self.Goal_pose)
        
        self.friend_pose = Pose()
        self.my_pose = Pose()
        self.rate = rospy.Rate(10)

    def Goal_pose(self, data):
        self.friend_pose = data
        self.move2goal()
        

    def update_pose(self, data):
        self.my_pose = data
        self.my_pose.x = round(self.my_pose.x, 4)
        self.my_pose.y = round(self.my_pose.y, 4)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.my_pose.x), 2) + pow((goal_pose.y - self.my_pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.my_pose.y, goal_pose.x - self.my_pose.x)

    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.my_pose.theta)

    def move2goal(self):
        tolerance = 0.1

        vm = Twist() # velocity_message

        while self.euclidean_distance(self.friend_pose) >= tolerance:
            vm.linear.x = self.linear_vel(self.friend_pose)
            vm.linear.y = 0
            vm.linear.z = 0
            vm.angular.x = 0
            vm.angular.y = 0
            vm.angular.z = self.angular_vel(self.friend_pose)
            self.v_publisher.publish(vm)
            self.rate.sleep()
        vm.linear.x = 0
        vm.angular.z = 0
        self.v_publisher.publish(vm)
        rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass