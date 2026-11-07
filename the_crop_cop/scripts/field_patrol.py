#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move_to_goal(x, y):
    # Create an action client that talks to move_base
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    rospy.loginfo("Waiting for move_base action server...")
    # Wait up to 5 seconds for the action server to become available
    client.wait_for_server(rospy.Duration(5.0))

    # Create the goal waypoint
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    # Set target coordinates
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    # Face forward (neutral orientation quaternion w=1.0)
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo(f"Sending goal to map coordinates: ({x}, {y})")
    
    # Send the goal to move_base (it will plan a path around obstacles automatically)
    client.send_goal(goal)
    
    # Wait for the robot to reach the goal
    finished = client.wait_for_result()
    
    if finished:
        state = client.get_state()
        rospy.loginfo(f"Successfully reached waypoint ({x}, {y}) with state: {state}")
        return True
    else:
        rospy.logwarn(f"Failed to reach waypoint ({x}, {y}) or timed out.")
        return False

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('crop_cop_field_patrol_action', anonymous=True)
        
        rospy.sleep(1)

        # Define crop field waypoints that are clear of direct collisions
        field_waypoints = [
            (1.0, 0.0),   # Row 1 start
            (1.0, 2.0),   # Row 1 end
            (2.0, 2.0),   # Move to Row 2
            (2.0, 0.0),   # Row 2 end
            (0.0, 0.0)    # Return home
        ]

        # Loop through each waypoint using move_base planner
        for waypoint in field_waypoints:
            rospy.loginfo(f"--- Navigating to waypoint: {waypoint} ---")
            success = move_to_goal(waypoint[0], waypoint[1])
            if not success:
                rospy.logerr("Navigation aborted or failed for this waypoint.")
                break
            rospy.sleep(1) # Pause at waypoint

        rospy.loginfo("Crop Field Patrol Complete via MoveBase Planner!")

    except rospy.ROSInterruptException:
        pass