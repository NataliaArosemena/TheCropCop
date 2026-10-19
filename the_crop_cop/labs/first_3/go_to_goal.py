#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, radians, degrees
from tf.transformations import euler_from_quaternion

class TurtleBotWaffleController:
    def __init__(self, use_controller=True):
        # Initialize the ROS node
        rospy.init_node('turtlebot3_waffle_goal_navigator', anonymous=True)

        # Flag to control whether to use P-controller or constant velocity
        self.use_controller = use_controller

        # Publisher for robot's velocity commands
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to robot's odometry data
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Control loop rate
        self.rate = rospy.Rate(10)

        # Initialize variables for robot's position and orientation
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.yaw_deg = 0.0  # Initialize yaw in degrees

        # Set the goal position (customize as needed)
        self.goal_x = 0.0  # Change this value to set a different goal
        self.goal_y = 0.0  # Change this value to set a different goal

        # Proportional control gains (used if P-controller is enabled)
        self.Kp_linear = 0.6   # Gain for linear velocity
        self.Kp_angular = 1.5  # Gain for angular velocity

        # Tolerance for reaching the goal
        self.goal_tolerance = 0.05  # Stop within 5 cm of the goal

    def odom_callback(self, msg):
        '''Callback function to update the robot's current position and orientation from /odom.'''
        # Extract the current position (x, y) from the odometry message
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extract the orientation in quaternion and convert to Euler (yaw)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.yaw = euler_from_quaternion(orientation_list)

        # Convert yaw to degrees and update yaw_deg
        self.yaw_deg = degrees(self.yaw)

        # Print the current position of the robot
        rospy.loginfo(f"Current Position -> x: {self.x:.2f} meters, y: {self.y:.2f} meters, yaw: {self.yaw_deg:.2f} degrees")

    def rotate_turtle(self, angular_speed_rads, angle_to_goal_deg):
        '''Rotate the turtle to face the goal'''
        rotate_cmd = Twist()

        # Set angular velocity directly in radians per second
        rotate_cmd.angular.z = angular_speed_rads

        rospy.loginfo(f"Rotating by {angle_to_goal_deg:.2f} degrees to face the goal")

        # TODO: Calculate time duration to rotate using the angle and angular speed
        # Hint: time = angle / angular speed
        time_duration = angle_to_goal_deg / angular_speed_rads

        # Get the start time
        start_time = rospy.get_time()

        # Rotate the turtle for the calculated time duration
        while rospy.get_time() - start_time < time_duration:
            self.pub.publish(rotate_cmd)
            self.rate.sleep()

        # Stop the rotation
        rotate_cmd.angular.z = 0.0
        self.pub.publish(rotate_cmd)

    def move_turtle(self, linear_speed, distance):
        '''Move the turtle forward towards the goal'''
        move_cmd = Twist()
        move_cmd.linear.x = linear_speed

        rospy.loginfo(f"Moving the turtle forward by {distance:.2f} meters")

        # TODO: Calculate time duration to move using the distance and linear speed
        time_duration = distance / linear_speed

        # Get the start time
        start_time = rospy.get_time()

        # Move the turtle forward for the calculated time duration
        while rospy.get_time() - start_time < time_duration:
            self.pub.publish(move_cmd)
            self.rate.sleep()

        # Stop the movement
        move_cmd.linear.x = 0.0
        self.pub.publish(move_cmd)
        
    def stop(self):
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.pub.publish(stop_cmd)
        
    def navigate_to_goal(self, x, y):
        '''Main function to navigate the TurtleBot to the goal'''
        self.goal_x = x
        self.goal_y = y
        # Step 1: Calculate the angle to the goal in degrees
        angle_to_goal = atan2(self.goal_y - self.y, self.goal_x - self.x)

        # TODO: Convert angle to degrees
        angle_to_goal_deg = angle_to_goal * (180 / 3.14)

        # Step 2: Calculate the distance to the goal
        distance_to_goal = sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)

        if self.use_controller:
            # P-Controller Mode
            rospy.loginfo("Navigating using P-Controller")
            self.p_controller_navigate(distance_to_goal, angle_to_goal_deg)
        else:
            # Non-Controlled Mode (Direct Navigation)
            self.rotate_turtle(0.26, angle_to_goal_deg)  # Rotate to face the goal at 0.5 rad/sec
            self.stop()
            rospy.sleep(1)
            self.move_turtle(1.82, distance_to_goal)  # Move towards the goal
        self.stop()

    def p_controller_navigate(self, distance_to_goal, angle_to_goal_deg):
        '''Navigate using a P-Controller'''
        rospy.loginfo(f"Navigating using P-Controller to goal at ({self.goal_x} meters, {self.goal_y} meters)")

        while not rospy.is_shutdown():
            # Recalculate distance and angle each iteration
            distance_to_goal = sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)
            desired_angle = atan2(self.goal_y - self.y, self.goal_x - self.x)
            
            # TODO: Convert desired angle to degrees
            desired_angle_deg = desired_angle * (180 / 3.14)

            angle_diff_deg = desired_angle_deg - self.yaw_deg

            move_cmd = Twist()

            if distance_to_goal > self.goal_tolerance:
                # TODO: Calculate the linear velocity based on the distance error
                move_cmd.linear.x = self.Kp_linear * distance_to_goal

                # TODO: Calculate the angular velocity based on the angle error
                move_cmd.angular.z = self.Kp_angular * (angle_diff_deg * (3.14 / 180))

                # Limit the velocities for safety (max linear and angular speed)
                move_cmd.linear.x = min(move_cmd.linear.x, 0.1)  # Max linear speed: 0.3 m/s
                move_cmd.angular.z = min(move_cmd.angular.z, 0.2)  # Max angular speed: 45 deg/s

                # Publish the velocity command to move the robot
                self.pub.publish(move_cmd)

                # Print current distance and angle difference to goal
                rospy.loginfo(f"Distance to goal: {distance_to_goal:.2f} meters, Angle difference: {angle_diff_deg:.2f} degrees")

            else:
                rospy.loginfo("Goal reached, stopping robot")
                self.pub.publish(Twist())  # Stop the robot
                break

            self.rate.sleep()

if __name__ == '__main__':
    try:
        # Instantiate the controller with a flag to use or not use the P-Controller
        controller = TurtleBotWaffleController(use_controller=True)  # Set to False to disable P-Controller
        controller.navigate_to_goal(3, 2)
    except rospy.ROSInterruptException:
        pass
