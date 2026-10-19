#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ColorFollower:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('color_follower')

        # Publisher to control the robot's movement
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to the camera feed
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Set the color range for blue in HSV
        self.lower_blue = np.array([100, 150, 0])
        self.upper_blue = np.array([140, 255, 255])

    def image_callback(self, data):
        # Convert the ROS image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # Convert the image to the HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Create a mask for the blue color
        mask_blue = cv2.inRange(hsv_image, self.lower_blue, self.upper_blue)

        # Find contours of the blue object
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours_blue:
            # Get the largest blue contour
            largest_contour = max(contours_blue, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Debugging: Print x, y, w, h to understand the size and position of the detected object
            rospy.loginfo(f"Bounding Box - x: {x}, y: {y}, w: {w}, h: {h}")

            # Calculate the center of the blue object
            object_center_x = x + w / 2
            image_center_x = cv_image.shape[1] / 2

            # Calculate the error (how far the object is from the center of the image)
            error_x = object_center_x - image_center_x

            # Create a Twist message to move the robot
            move_cmd = Twist()

            # Debugging: Print the error in the x-axis (how far off the object is from the center)
            rospy.loginfo(f"Error in X: {error_x}")

            # TODO 1: Implement proportional control for turning based on error_x
            Kp = -0.001
            move_cmd.angular.z = Kp * error_x

            # TODO 2: Check if the width is less than 500 (to determine how close the object is)
            if(w < 500):
                move_cmd.linear.x = 0.2
            else:
                move_cmd.linear.x = 0.0
        
            # TODO 3: Publish the move command
            self.pub.publish(move_cmd)

            # Optional: Display the image with the detected object
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.imshow('Object Tracker', cv_image)
            cv2.waitKey(1)
        else:
            # If no blue object is detected, stop the robot
            move_cmd = Twist()
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.pub.publish(move_cmd)
            rospy.loginfo("No object detected - stopping")

if __name__ == '__main__':
    try:
        follower = ColorFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
