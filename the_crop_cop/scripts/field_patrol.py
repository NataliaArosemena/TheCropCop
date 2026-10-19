#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, radians, degrees
from tf.transformations import euler_from_quaternion

class CropCopPatrolController:
    def __init__(self, use_controller=True):
        # Initialize the ROS node
        rospy.init_node('crop_cop_field_patrol', anonymous=True)

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

        # Goal variables
        self.goal_x = 0.0
        self.goal_y = 0.0

        # Proportional control gains
        self.Kp_linear = 0.6   # Gain for linear velocity
        self.Kp_angular = 1.5  # Gain for angular velocity

        # Tolerance for reaching each crop waypoint
        self.goal_tolerance = 0.1  # Stop within 10 cm of the goal waypoint

    def odom_callback(self, msg):
        '''Callback function to update the robot's current position and orientation from /odom.'''
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.yaw = euler_from_quaternion(orientation_list)

        self.yaw_deg = degrees(self.yaw)

    def rotate_turtle(self, angular_speed_rads, angle_to_goal_deg):
        '''Rotate the turtle to face the goal'''
        rotate_cmd = Twist()
        rotate_cmd.angular.z = angular_speed_rads

        rospy.loginfo(f"Rotating by {angle_to_goal_deg:.2f} degrees")
        time_duration = angle_to_goal_deg / angular_speed_rads

        start_time = rospy.get_time()
        while rospy.get_time() - start_time < time_duration:
            self.pub.publish(rotate_cmd)
            self.rate.sleep()

        rotate_cmd.angular.z = 0.0
        self.pub.publish(rotate_cmd)

    def move_turtle(self, linear_speed, distance):
        '''Move the turtle forward towards the goal'''
        move_cmd = Twist()
        move_cmd.linear.x = linear_speed

        rospy.loginfo(f"Moving forward by {distance:.2f} meters")
        time_duration = distance / linear_speed

        start_time = rospy.get_time()
        while rospy.get_time() - start_time < time_duration:
            self.pub.publish(move_cmd)
            self.rate.sleep()

        move_cmd.linear.x = 0.0
        self.pub.publish(move_cmd)

    def stop(self):
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.pub.publish(stop_cmd)

    def navigate_to_goal(self, x, y):
        '''Navigate the Crop Cop to a specific coordinate'''
        self.goal_x = x
        self.goal_y = y

        angle_to_goal = atan2(self.goal_y - self.y, self.goal_x - self.x)
        angle_to_goal_deg = degrees(angle_to_goal)
        distance_to_goal = sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)

        if self.use_controller:
            self.p_controller_navigate(distance_to_goal, angle_to_goal_deg)
        else:
            self.rotate_turtle(0.26, angle_to_goal_deg)
            self.stop()
            rospy.sleep(1)
            self.move_turtle(0.18, distance_to_goal)
        self.stop()

    def p_controller_navigate(self, distance_to_goal, angle_to_goal_deg):
        '''Navigate using P-Controller'''
        rospy.loginfo(f"Heading to waypoint at ({self.goal_x}, {self.goal_y})")

        while not rospy.is_shutdown():
            distance_to_goal = sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)
            desired_angle = atan2(self.goal_y - self.y, self.goal_x - self.x)
            desired_angle_deg = degrees(desired_angle)

            angle_diff_deg = desired_angle_deg - self.yaw_deg

            # Normalize angle difference to stay between -180 and 180 degrees
            while angle_diff_deg > 180:
                angle_diff_deg -= 360
            while angle_diff_deg < -180:
                angle_diff_deg += 360

            move_cmd = Twist()

            if distance_to_goal > self.goal_tolerance:
                move_cmd.linear.x = self.Kp_linear * distance_to_goal
                move_cmd.angular.z = self.Kp_angular * radians(angle_diff_deg)

                # Velocity limits for safety
                move_cmd.linear.x = min(move_cmd.linear.x, 0.15)
                move_cmd.angular.z = max(min(move_cmd.angular.z, 0.4), -0.4)

                self.pub.publish(move_cmd)
                rospy.loginfo(f"Pos: ({self.x:.2f}, {self.y:.2f}) | Dist to Waypoint: {distance_to_goal:.2f}m")
            else:
                rospy.loginfo(f"Reached Waypoint ({self.goal_x}, {self.goal_y})!")
                self.pub.publish(Twist())  # Stop briefly
                rospy.sleep(1)  # Pause for 1 second at each waypoint (e.g., inspecting crops)
                break

            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = CropCopPatrolController(use_controller=True)

        # Wait briefly for odometry data to come in before starting
        rospy.sleep(1)

        # Define crop field waypoints [(x1, y1), (x2, y2), ...]
        field_waypoints = [
            (1.0, 0.0),   # Row 1 start
            (1.0, 2.0),   # Row 1 end
            (2.0, 2.0),   # Move to Row 2
            (2.0, 0.0),   # Row 2 end
            (0.0, 0.0)    # Return to home base
        ]

        # Loop through each waypoint in order
        for waypoint in field_waypoints:
            rospy.loginfo(f"--- Starting navigation to waypoint: {waypoint} ---")
            controller.navigate_to_goal(waypoint[0], waypoint[1])

        rospy.loginfo("Crop Field Patrol Complete!")

    except rospy.ROSInterruptException:
        pass