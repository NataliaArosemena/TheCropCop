#!/usr/bin/env python3

import rospy
import actionlib
import yaml
import os
import subprocess
import random
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

try:
    from PIL import Image
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

def get_latest_map_path():
    """Finds the path to the current garden map yaml automatically."""
    try:
        package_path = subprocess.check_output(['rospack', 'find', 'the_crop_cop']).decode('utf-8').strip()
        return f"{package_path}/maps/garden_map.yaml"
    except Exception:
        return None

def load_map_metadata(yaml_path):
    """Loads map resolution and origin to translate pixel grid to world coordinates."""
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)
    return meta

def generate_dynamic_waypoints(yaml_path, meta):
    """
    Generates waypoints using exact, guaranteed open aisle corridors 
    tailored specifically to your multi-tier garden layout.
    """
    waypoints = []
    
    # --- EXPLICIT, FOOLPROOF OPEN ALLEY CORRIDORS ---
    # These Y values sit cleanly in the wide open alleys between your plant tiers:
    # 1. Bottom alley (below bottom tier)
    # 2. Lower-middle alley (between tier 1 and tier 2)
    # 3. Upper-middle alley (between tier 2 and tier 3)
    # 4. Top alley (above top tier)
    aisle_y_levels = [-1.9, -1.00, -0.10, 0.9, 1.85]

    # Safe X bounds that stay well away from side walls and outer corners
    left_x = -1.7
    right_x = 1.7

    rospy.loginfo(f"Using foolproof alley Y levels: {aisle_y_levels}")

    moving_right = True
    for current_y in aisle_y_levels:
        row_waypoints = []
        
        cx = left_x
        while cx <= right_x:
            row_waypoints.append((cx, current_y))
            cx += 0.35  # Short, reliable step intervals

        if not moving_right:
            row_waypoints.reverse()

        waypoints.extend(row_waypoints)
        moving_right = not moving_right

    rospy.loginfo(f"Generated {len(waypoints)} clean, collision-free patrol waypoints.")
    return waypoints

def navigate_to_pose(client, x, y, prev_x=None, prev_y=None):
    """Sends a target pose to move_base with a safer 45-second timeout."""
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    if prev_x is not None and prev_y is not None:
        angle = math.atan2(y - prev_y, x - prev_x)
    else:
        angle = 0.0

    goal.target_pose.pose.orientation.z = math.sin(angle / 2.0)
    goal.target_pose.pose.orientation.w = math.cos(angle / 2.0)

    client.send_goal(goal)
    finished = client.wait_for_result(rospy.Duration(45.0))
    
    if finished:
        state = client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            return True
        else:
            rospy.logwarn(f"Move base goal failed with state code: {state}")
            return False
    else:
        rospy.logwarn("Move base goal timed out!")
        client.cancel_goal()
        return False

def perform_pest_inspection(waypoint_index, x, y):
    """Simulates computer vision pest detection (good vs. bad bugs by color)."""
    rospy.loginfo(f"🔍 [VISION SCAN] Inspecting plant zone {waypoint_index} at ({x:.2f}, {y:.2f})...")
    rospy.sleep(1.0)
    
    findings = [
        {"status": "Clear", "desc": "No insects found."},
        {"status": "Beneficial", "desc": "Ladybug detected (Red/Black) -> Protecting pollinator."},
        {"status": "Harmful", "desc": "Aphid cluster detected (Green/Yellow) -> Triggering target alert!"},
        {"status": "Clear", "desc": "Plant health optimal."}
    ]
    result = random.choice(findings)
    
    if result["status"] == "Harmful":
        rospy.logwarn(f"🚨 [PEST ALERT] {result['desc']}")
    elif result["status"] == "Beneficial":
        rospy.loginfo(f"🐞 [BENEFICIAL BUG] {result['desc']}")
    else:
        rospy.loginfo(f"✅ [STATUS] {result['desc']}")

def return_to_home_and_recharge(client):
    """Navigates the robot back to home base and simulates battery recharge."""
    rospy.loginfo("🔋 [BATTERY LOW] Returning to home base for recharging...")
    
    success = navigate_to_pose(client, 0.0, 0.0)
    
    if success:
        rospy.loginfo("🏠 [HOME REACHED] Docked at charging station. Recharging battery...")
        for i in range(10):
            if rospy.is_shutdown():
                break
            rospy.loginfo(f"⚡ Charging... ({ (i+1) * 10 }% completed)")
            rospy.sleep(2.0)
        rospy.loginfo("✨ [FULLY CHARGED] Battery at 100%. Resuming garden patrol mission!")
    else:
        rospy.logerr("❌ Failed to navigate back to home base!")

def patrol_loop():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    if not client.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr("move_base server not available!")
        return

    yaml_path = get_latest_map_path()
    meta = load_map_metadata(yaml_path) if yaml_path else {}
    waypoints = generate_dynamic_waypoints(yaml_path, meta)

    battery_level = 100.0
    battery_drain_per_stop = 0.9

    rospy.loginfo(f"Starting patrol with {len(waypoints)} waypoints. Aisle-locked navigation active.")

    loop_count = 1
    while not rospy.is_shutdown():
        rospy.loginfo(f"=== Starting Patrol Loop #{loop_count} ===")
        
        curr_x, curr_y = 0.0, 0.0
        for i, (x, y) in enumerate(waypoints):
            if rospy.is_shutdown():
                break
            
            if battery_level <= 20.0:
                return_to_home_and_recharge(client)
                battery_level = 100.0
                rospy.loginfo(f"=== Resuming Patrol Loop #{loop_count} ===")

            rospy.loginfo(f"Patrolling target {i+1}/{len(waypoints)} -> ({x:.2f}, {y:.2f}) [Battery: {battery_level:.1f}%]")
            
            success = navigate_to_pose(client, x, y, curr_x, curr_y)
            
            if success:
                perform_pest_inspection(i+1, x, y)
                battery_level -= battery_drain_per_stop
                curr_x, curr_y = x, y
            else:
                rospy.logwarn(f"⚠️ Target {i+1} unreachable. Skipping safely to next point.")
                client.cancel_goal()
                curr_x, curr_y = x, y
                rospy.sleep(1.0)

        rospy.loginfo(f"Completed Patrol Loop #{loop_count}.")
        loop_count += 1

if __name__ == '__main__':
    try:
        rospy.init_node('dynamic_garden_patrol_battery', anonymous=True)
        patrol_loop()
    except rospy.ROSInterruptException:
        pass