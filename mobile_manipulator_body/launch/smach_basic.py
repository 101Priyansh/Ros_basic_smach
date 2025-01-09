#!/usr/bin/env python

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt


class GoalNavigation:
    def __init__(self):
        rospy.init_node('goal_navigation', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/robot_base_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/robot_base_velocity_controller/odom', Odometry, self.odom_callback)

        self.current_position = {'x': 0.0, 'y': 0.0}
        self.goal_position = {'x': 6.0, 'y': 0.0}
        self.threshold = 0.2
        self.reached_goal = False
        self.rate = rospy.Rate(10)

    def odom_callback(self, msg):
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y

    def move_to_goal(self):
        velocity_msg = Twist()
        dist_prev = None
        while not self.reached_goal:
            # Calculate distance to the goal
            distance_to_goal = self.goal_position['x'] - self.current_position['x']
            #sqrt(pow(, 2) + pow(self.goal_position['y'] - self.current_position['y'], 2))
           
            
            # rospy.loginfo("Current Position: x = %.2f, y = %.2f", self.current_position['x'], self.current_position['y'])
            rospy.loginfo("Distance to Goal: %.2f", distance_to_goal)
            # Check if the robot is moving away from the goal
            if dist_prev is not None and distance_to_goal > dist_prev + 0.01:
                rospy.loginfo("Distance prev: %.2f", dist_prev)
                rospy.loginfo("Moving away from goal! Aborting goal approach.")
                return 'moving_away'

            dist_prev = distance_to_goal  # Update previous distance

            if distance_to_goal <= self.threshold:
                rospy.loginfo("Goal Reached!")
                self.reached_goal = True
                velocity_msg.linear.x = 0.0
                velocity_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(velocity_msg)
                return 'goal_reached'
            else:
                velocity_msg.linear.x = 0.3 *distance_to_goal # Move towards the goal
                velocity_msg.angular.z = 0.0  # No turning, straight line
                self.cmd_vel_pub.publish(velocity_msg)

            self.rate.sleep()
        #rospy.spin()

    def move_in_circle(self):
        # Move in a circle with constant velocity
        velocity_msg = Twist()
        velocity_msg.linear.x = 0.4  # Forward speed
        velocity_msg.angular.z = 0.3  # Turning speed
        duration = 10.0
        start_time = rospy.Time.now()
        rospy.loginfo("Moving in circle in...")
        # Publish circular motion for 5 seconds
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < duration:
            self.cmd_vel_pub.publish(velocity_msg)
            rate.sleep()
            # Stop after circular motion is done
        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(velocity_msg)
        rospy.loginfo("Circular motion completed.")


class MoveToGoal(smach.State):
    def __init__(self, navigator):
        smach.State.__init__(self, outcomes=['goal_reached', 'move_in_circle'])
        self.navigator = navigator

    def execute(self, userdata):
        rospy.loginfo("Moving to goal...")
        result = self.navigator.move_to_goal()
        rospy.sleep(1)
        # If goal is reached, return 'goal_reached'
        if result =='goal_reached':
            return 'move_in_circle'
        if result =='moving_away':
            rospy.loginfo("Transitioning to MoveInCircle due to distance increase.")
            return 'move_in_circle' 
        else:
            return 'goal_reached'


class MoveInCircle(smach.State):
    def __init__(self, navigator):
        smach.State.__init__(self, outcomes=['circle_done'])
        self.navigator = navigator

    def execute(self, userdata):
        rospy.loginfo("Initiating circular motion.")
        self.navigator.move_in_circle()
        rospy.loginfo("Moving in circle...")
        return 'circle_done'


def main():
    navigator = GoalNavigation()

    # Create the state machine
    sm = smach.StateMachine(outcomes=['goal_reached', 'circle_done'])

    with sm:
        # Define starting with MOVE_TO_GOAL state
        smach.StateMachine.add('MOVE_TO_GOAL', MoveToGoal(navigator), 
                               transitions={'goal_reached': 'goal_reached', 
                                            'move_in_circle': 'MOVE_IN_CIRCLE'})
        smach.StateMachine.add('MOVE_IN_CIRCLE', MoveInCircle(navigator), 
                               transitions={'circle_done': 'goal_reached'})

    # Execute the state machine
    sis = smach_ros.IntrospectionServer('server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    sis.stop()

    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
