#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class DrawM:
    def __init__(self):
        rospy.init_node('turtle_triangle', anonymous=True)

        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        self.current_pose = Pose()
        self.rate = rospy.Rate(10)

    def pose_callback(self, data):
        self.current_pose = data

    def move_to_goal(self, goal_x, goal_y):
        vel_msg = Twist()

        Kp_linear = 2.0
        Kp_angular = 10.28 

        while not rospy.is_shutdown():
            distance = math.sqrt((goal_x - self.current_pose.x)**2 + (goal_y - self.current_pose.y)**2)
            angle_to_goal = math.atan2(goal_y - self.current_pose.y, goal_x - self.current_pose.x)
            angle_to_goal = round(angle_to_goal, 2)
            angle_diff = angle_to_goal - self.current_pose.theta

            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            if abs(angle_diff) > math.pi/3600:
            	vel_msg.linear.x = 0.3 * distance
            	vel_msg.angular.z = Kp_angular * angle_diff
            
            if abs(angle_diff) > math.pi/900:
            	vel_msg.linear.x = 0.05 * distance
            	vel_msg.angular.z = Kp_angular * angle_diff *1.25
            
            
            else:
            	vel_msg.linear.x = Kp_linear * distance
            	#vel_msg.angular.z = Kp_angular * angle_diff

            self.vel_pub.publish(vel_msg)

            if distance < 0.1:
                break

            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)

    def draw_m(self):
        points = [
    		(5.45, 5.45),
    		(5.30, 5.45),
    		(4.86, 2.86),
    		(5.12, 2.86),
    		(4.81, 1.55),
    		(4.78, 1.55),
    		(4.50, 1.55),
    		(4.00, 1.55),
    		(3.31, 1.55),
    		(3.83, 4.19),
    		(2.85, 2.75),
    		(2.35, 4.19),
    		(1.75, 1.55),
    		(1.65, 1.55),
    		(1.25, 1.55),
    		(0.75, 1.55),
    		(0.10, 1.55),
    		(0.45, 2.86),
    		(0.71, 2.86),
    		(1.17, 5.45),
    		(0.92, 5.45),
    		(1.20, 6.83),
    		(1.50, 6.83),
    		(2.00, 6.83),
    		(2.5, 6.83),
    		(2.79, 6.83),
    		(3.13, 5.49),
    		(3.98, 6.83),
    		(4.08, 6.83),
    		(4.5, 6.83),
    		(5.25, 6.83),
    		(5.75, 6.83),
    		(5.45, 5.45)
	]

        for point in points:
            self.move_to_goal(point[0], point[1])
            rospy.sleep(1)  

if __name__ == '__main__':
    try:
        turtle_cont = DrawM()
        rospy.sleep(1)
        turtle_cont.draw_m()

    except rospy.ROSInterruptException:
        pass
