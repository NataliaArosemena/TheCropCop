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
    Analyzes plant bed clusters (rows vs columns) to determine layout orientation,
    hugs the beds tightly on outer tracks, and generates collision-free waypoints.
    """
    waypoints = []
    
    base_dir = os.path.dirname(yaml_path)
    image_filename = meta.get('image', 'garden_map.pgm')
    image_path = os.path.join(base_dir, image_filename) if not os.path.isabs(image_filename) else image_filename

    if not HAS_PIL or not os.path.exists(image_path):
        rospy.logwarn("PIL missing or PGM not found. Falling back to default waypoints.")
        return [(-1.0, -1.0), (1.0, -1.0)]

    img = Image.open(image_path).convert('L')
    width, height = img.size
    pixels = img.load()

    resolution = meta['resolution']
    origin_x = meta['origin'][0]
    origin_y = meta['origin'][1]

    def px_to_world(px, py):
        wx = origin_x + (px * resolution)
        wy = origin_y + ((height - py) * resolution)
        return wx, wy

    # 1. Collect all obstacle pixels within the inner garden room
    obs_x, obs_y = [], []
    for py in range(height):
        for px in range(width):
            if pixels[px, py] < 100:
                wx, wy = px_to_world(px, py)
                obs_x.append(wx)
                obs_y.append(wy)

    if not obs_x:
        rospy.logerr("Could not detect any obstacles in map image!")
        return [(0.0, 0.0)]

    wall_min_x, wall_max_x = min(obs_x), max(obs_x)
    wall_min_y, wall_max_y = min(obs_y), max(obs_y)

    # Isolate internal plant boxes (strip away outer walls)
    internal_x = [wx for wx, wy in zip(obs_x, obs_y) if (wall_min_x + 0.3 < wx < wall_max_x - 0.3) and (wall_min_y + 0.3 < wy < wall_max_y - 0.3)]
    internal_y = [wy for wx, wy in zip(obs_x, obs_y) if (wall_min_x + 0.3 < wx < wall_max_x - 0.3) and (wall_min_y + 0.3 < wy < wall_max_y - 0.3)]

    if not internal_y or not internal_x:
        rospy.logerr("Could not isolate plant beds from walls!")
        return [(0.0, 0.0)]

    # 2. Cluster plant boxes into Rows (Y-axis groups) and Columns (X-axis groups)
    unique_ys = sorted(list(set([round(y, 2) for y in internal_y])))
    row_clusters = []
    if unique_ys:
        cur_row = [unique_ys[0]]
        for y in unique_ys[1:]:
            if y - cur_row[-1] <= 0.25:
                cur_row.append(y)
            else:
                row_clusters.append((min(cur_row), max(cur_row)))
                cur_row = [y]
        row_clusters.append((min(cur_row), max(cur_row)))
    significant_rows = [r for r in row_clusters if (r[1] - r[0]) >= 0.15]
    significant_rows.sort(key=lambda r: r[0], reverse=True) # Top to bottom

    unique_xs = sorted(list(set([round(x, 2) for x in internal_x])))
    col_clusters = []
    if unique_xs:
        cur_col = [unique_xs[0]]
        for x in unique_xs[1:]:
            if x - cur_col[-1] <= 0.3:
                cur_col.append(x)
            else:
                col_clusters.append((min(cur_col), max(cur_col)))
                cur_col = [x]
        col_clusters.append((min(cur_col), max(cur_col)))
    significant_cols = [c for c in col_clusters if (c[1] - c[0]) >= 0.15]

    # 3. Detect layout structure based on bed counts (Columns vs Rows)
    num_rows = len(significant_rows)
    num_cols = len(significant_cols)
    is_vertical_layout = num_cols > num_rows

    rospy.loginfo(f"Bed Cluster Analysis -> Plant Rows (Tiers): {num_rows}, Plant Columns: {num_cols}. Layout Strategy: {'VERTICAL COLUMNS' if is_vertical_layout else 'HORIZONTAL ROWS'}")

    waypoints = []
    moving_forward = True

    if not is_vertical_layout and significant_rows:
        # --- HORIZONTAL SWEEPING (Rows stacked vertically, sweeping left/right) ---
        aisle_y_levels = []
        
        # Top outer row pulled tightly to the top plant bed
        aisle_y_levels.append(significant_rows[0][1] + 0.30)

        # Gaps between plant tiers
        for i in range(len(significant_rows) - 1):
            bottom_of_upper = significant_rows[i][0]
            top_of_lower = significant_rows[i+1][1]
            aisle_y_levels.append((bottom_of_upper + top_of_lower) / 2.0)

        # Bottom outer row pulled tightly to the bottom plant bed
        aisle_y_levels.append(significant_rows[-1][0] - 0.30)

        left_x = min(internal_x) + 0.35
        right_x = max(internal_x) - 0.35

        for current_y in aisle_y_levels:
            row_waypoints = []
            cx = left_x
            while cx <= right_x:
                row_waypoints.append((cx, current_y))
                cx += 0.35

            if not moving_forward:
                row_waypoints.reverse()
            waypoints.extend(row_waypoints)
            moving_forward = not moving_forward

    else:
        # --- VERTICAL SWEEPING (Columns side-by-side, sweeping up/down) ---
        aisle_x_levels = []
        if significant_cols:
            aisle_x_levels.append(significant_cols[0][0] - 0.30)
            for i in range(len(significant_cols) - 1):
                right_of_left = significant_cols[i][1]
                left_of_right = significant_cols[i+1][0]
                aisle_x_levels.append((right_of_left + left_of_right) / 2.0)
            aisle_x_levels.append(significant_cols[-1][1] + 0.30)

        bottom_y = significant_rows[-1][0] - 0.35 if significant_rows else wall_min_y + 0.5
        top_y = significant_rows[0][1] + 0.35 if significant_rows else wall_max_y - 0.5

        for current_x in aisle_x_levels:
            col_waypoints = []
            cy = bottom_y
            while cy <= top_y:
                col_waypoints.append((current_x, cy))
                cy += 0.35

            if not moving_forward:
                col_waypoints.reverse()
            waypoints.extend(col_waypoints)
            moving_forward = not moving_forward

    rospy.loginfo(f"Generated {len(waypoints)} bed-hugging, cluster-aware waypoints.")
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