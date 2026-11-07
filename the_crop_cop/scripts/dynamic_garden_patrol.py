#!/usr/bin/env python3

import rospy
import actionlib
import yaml
import os
import subprocess
import random
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
    Isolates internal plant boxes by finding all obstacles and filtering 
    out the outer perimeter walls, then generates map-aligned inspection stops.
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

    # Find ALL obstacle pixels (both outer walls and plant boxes are black < 100)
    all_obs_x = []
    all_obs_y = []

    for y in range(height):
        for x in range(width):
            if pixels[x, y] < 100:
                wx = origin_x + (x * resolution)
                wy = origin_y + ((height - y) * resolution)
                all_obs_x.append(wx)
                all_obs_y.append(wy)

    if not all_obs_x:
        rospy.logerr("Could not detect any obstacles in map image!")
        return [(0.0, 0.0)]

    # 1. Determine outer room boundaries
    outer_min_x, outer_max_x = min(all_obs_x), max(all_obs_x)
    outer_min_y, outer_max_y = min(all_obs_y), max(all_obs_y)

    # 2. Filter for INNER obstacles (the plant boxes inside the room, away from perimeter walls)
    plant_box_x = []
    plant_box_y = []
    for wx, wy in zip(all_obs_x, all_obs_y):
        # Exclude pixels sitting right on the outer boundary walls
        if (outer_min_x + 0.4 < wx < outer_max_x - 0.4) and (outer_min_y + 0.4 < wy < outer_max_y - 0.4):
            plant_box_x.append(wx)
            plant_box_y.append(wy)

    if not plant_box_x:
        rospy.logerr("Could not isolate internal plant boxes from walls!")
        return [(0.0, 0.0)]

    # Get true boundaries of the plant cluster region
    plants_min_x, plants_max_x = min(plant_box_x), max(plant_box_x)
    plants_min_y, plants_max_y = min(plant_box_y), max(plant_box_y)

    rospy.loginfo(f"Detected plant box cluster bounds: X[{plants_min_x:.2f} to {plants_max_x:.2f}], Y[{plants_min_y:.2f} to {plants_max_y:.2f}]")

    # --- GENERATE MAP-ALIGNED INSPECTION STOPS ---
    # Position the robot's sweep rows directly through the aisles alongside the detected plant boxes
    aisle_spacing_y = 0.6
    current_y = plants_min_y - 0.3
    moving_right = True

    # Generate horizontal sweep points spanning across the plant width
    while current_y <= plants_max_y + 0.3:
        row_points = []
        # Create stop intervals across the row where individual plant boxes sit
        step = 0.5
        cx = plants_min_x - 0.2
        while cx <= plants_max_x + 0.2:
            row_points.append((cx, current_y))
            cx += step

        if not moving_right:
            row_points.reverse()

        waypoints.extend(row_points)
        current_y += aisle_spacing_y
        moving_right = not moving_right

    rospy.loginfo(f"Generated {len(waypoints)} plant-aligned inspection waypoints.")
    return waypoints

def perform_crop_inspection(waypoint_index, x, y):
    """Simulates a computer vision color-detection scan to classify pests and insects."""
    rospy.loginfo(f"🔍 [VISION SCAN] Analyzing crop zone {waypoint_index} at ({x:.2f}, {y:.2f})...")
    
    # Simulate camera shutter / frame processing delay
    rospy.sleep(1.2)
    
    # Simulate a computer vision color classification result
    # (In a real deployment, this would process RGB frames from your camera topic)
    scan_results = [
        {"status": "Clear", "pest": None, "action": "None"},
        {"status": "Beneficial Insect Detected", "pest": "Ladybug (Red/Black)", "action": "Protect / Leave Alone"},
        {"status": "Harmful Pest Detected", "pest": "Aphid Cluster (Green/Yellow)", "action": "Trigger Target Alert"},
        {"status": "Harmful Pest Detected", "pest": "Caterpillar (Dark Green)", "action": "Trigger Target Alert"},
        {"status": "Optimal Growth", "pest": None, "action": "None"}
    ]
    
    finding = random.choice(scan_results)
    
    if "Harmful" in finding["status"]:
        rospy.logwarn(f"🚨 [PEST ALERT] Harmful bug identified! Type: {finding['pest']} | Recommended Action: {finding['action']}")
    elif "Beneficial" in finding["status"]:
        rospy.loginfo(f"🐞 [BENEFICIAL] Good insect found: {finding['pest']} | Status: {finding['action']}")
    else:
        rospy.loginfo(f"✅ [STATUS] Aisle clear. Plant health optimal.")

    rospy.sleep(0.5)

def patrol_loop():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    if not client.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr("move_base server not available!")
        return

    yaml_path = get_latest_map_path()
    if not yaml_path or not os.path.exists(yaml_path):
        rospy.logerr("Could not locate garden_map.yaml. Make sure a map is saved!")
        return

    meta = load_map_metadata(yaml_path)
    waypoints = generate_dynamic_waypoints(yaml_path, meta)

    rospy.loginfo(f"Generated {len(waypoints)} dynamic patrol waypoints from map file. Starting continuous loop...")

    loop_count = 1
    while not rospy.is_shutdown():
        rospy.loginfo(f"=== Starting Patrol Loop #{loop_count} ===")
        for i, (x, y) in enumerate(waypoints):
            if rospy.is_shutdown():
                break
                
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.w = 1.0

            rospy.loginfo(f"Patrolling dynamic target {i+1}/{len(waypoints)} -> ({x:.2f}, {y:.2f})")
            client.send_goal(goal)
            
            finished = client.wait_for_result()
            if finished:
                # Trigger the simulated computer vision / sensor inspection routine
                perform_crop_inspection(i+1, x, y)
            else:
                rospy.logwarn(f"Waypoint ({x:.2f}, {y:.2f}) unreachable or blocked, moving to next.")

        rospy.loginfo(f"Completed Patrol Loop #{loop_count}. Restarting...")
        loop_count += 1

if __name__ == '__main__':
    try:
        rospy.init_node('dynamic_garden_patrol_inspection', anonymous=True)
        patrol_loop()
    except rospy.ROSInterruptException:
        pass