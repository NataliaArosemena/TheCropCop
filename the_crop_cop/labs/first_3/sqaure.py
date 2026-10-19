#!/usr/bin/env python3

# Import necessary ROS libraries
import rospy  # Python client library for ROS
from geometry_msgs.msg import Twist  # Message type for controlling robot motion
import math  # For converting degrees to radians
rospy.init_node('rotate_turtle_node', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def rotate_turtle(angle_degrees, angular_speed_rads):
    
    #rospy.init_node('rotate_turtle_node', anonymous=True)


    # Create a Twist message object to hold velocity commands
    rotate_cmd = Twist()

    # Convert the input angle from degrees to radians
    angle_radians = math.radians(angle_degrees)

    # Calculate the required time for the rotation using the formula time = angle / speed
    time_duration = angle_radians / angular_speed_rads

    # Set the angular speed in the Twist message
    rotate_cmd.linear.x = 0.0  # No linear movement (we're only rotating)
    rotate_cmd.angular.z = angular_speed_rads  # Set the angular velocity for rotation

    # Set the rate at which to send messages (10 messages per second)
    rate = rospy.Rate(200)

    # Inform the user that the node has started
    rospy.loginfo(f"Rotating the turtle by {angle_degrees} degrees at {angular_speed_rads} radians/second...")

    # Get the start time (in seconds) using rospy.get_time()
    start_time = rospy.get_time()

    # Rotate the turtle for the calculated time duration
    while rospy.get_time() - start_time < time_duration:
        # Publish the Twist message to the /cmd_vel topic
        pub.publish(rotate_cmd)

        # Sleep to maintain the desired rate
        rate.sleep()

    # Stop the turtle after rotating the desired angle
    rotate_cmd.angular.z = 0.0  # Stop rotating
    pub.publish(rotate_cmd)

    rospy.loginfo(f"Turtle has stopped rotating after {angle_degrees} degrees.")

def move_turtle(linear_speed, distance):

    # Create a Twist message object to hold velocity commands
    move_cmd = Twist()

    # Set the linear velocity for forward movement
    move_cmd.linear.x = linear_speed  
    move_cmd.angular.z = 0.0  # No rotation (angular velocity)

    # Calculate the time duration using time = distance / speed
    time_duration = distance / linear_speed

    # Set the rate at which to send messages (10 messages per second)
    rate = rospy.Rate(200)

    # Inform the user that the node has started
    rospy.loginfo(f"Moving the turtle forward by {distance} meters at {linear_speed} m/s...")

    # Get the start time (in seconds) using rospy.get_time()
    start_time = rospy.get_time()

    # Move the turtle for the calculated time duration
    while rospy.get_time() - start_time < time_duration:
        # Publish the Twist message to the /cmd_vel topic
        pub.publish(move_cmd)

        # Sleep to maintain the desired rate
        rate.sleep()

    # Stop the turtle after moving the desired distance
    move_cmd.linear.x = 0.0  # Stop moving forward
    pub.publish(move_cmd)

    rospy.loginfo("Turtle has stopped moving.")

if __name__ == '__main__':
    try:
        # Run the move_turtle function with specified linear speed and distance
        move_turtle(0.1, 1.0)
        rotate_turtle(90, 0.07)
        move_turtle(0.1, 1.0)
        rotate_turtle(90, 0.07)
        move_turtle(0.1, 1.0)
        rotate_turtle(90, 0.07)
        move_turtle(0.1, 1.0)
        rotate_turtle(90, 0.07)
        move_turtle(0.1, 1.0)
     
    except rospy.ROSInterruptException:
        # Handle exceptions and clean up if needed
        pass

