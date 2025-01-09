#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, pow

class GoalNavigation:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('goal_navigation', anonymous=True)


        ##PUB SUB format

        # Publisher to /cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/robot_base_velocity_controller/cmd_vel', Twist, queue_size=10)

        # Subscriber to /odom
        rospy.Subscriber('/robot_base_velocity_controller/odom', Odometry, self.odom_callback)



        # Initialise Positions:
        self.current_position = {'x': 0.0, 'y': 0.0}
        self.goal_position = {'x': 6.0, 'y': 0.0} 
        self.threshold = 0.2

        self.reached_goal = False
        self.rate = rospy.Rate(10)  # rate of publishing

    #required for subscriber
    def odom_callback(self, msg):
        # Extract position from Odometry message
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y


    # logic to reach goal

    def move_to_goal(self):
        velocity_msg = Twist()

        while not rospy.is_shutdown() and not self.reached_goal:
            
            
            # Calculate distance to goal
            distance_to_goal = self.goal_position['x'] - self.current_position['x']

            rospy.loginfo("Current Position: goal_x = %.2f, x = %.2f, y = %.2f", self.goal_position['x'],self.current_position['x'], self.current_position['y'])
            rospy.loginfo("Distance to Goal: %.2f", distance_to_goal)

            
            # Check if robot is within threshold?

            if distance_to_goal <= self.threshold:
                rospy.loginfo("Goal Reached!")
                
                
                self.reached_goal = True
                velocity_msg.linear.x = 0.0
                velocity_msg.angular.z = 0.0


                ##publishing again here to STOP!
                self.cmd_vel_pub.publish(velocity_msg)
                break


            # Set velocity commands to move toward the goal
            velocity_msg.linear.x = 0.3 * distance_to_goal  # Proportional control
            velocity_msg.angular.z = 0.0  # straight-line motion
            self.cmd_vel_pub.publish(velocity_msg)

            self.rate.sleep()

        rospy.spin()

if __name__ == '__main__':
    try:
        navigator = GoalNavigation()
        navigator.move_to_goal()
    except rospy.ROSInterruptException:
        pass
