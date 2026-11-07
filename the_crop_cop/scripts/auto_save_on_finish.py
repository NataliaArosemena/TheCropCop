#!/usr/bin/env python3
import os
import subprocess
import time
import rosnode
import rospy


def check_explore_status():
  rospy.init_node('auto_save_on_finish', anonymous=True)
  rospy.loginfo('Auto-save monitor active. Waiting for exploration to finish...')

  # Give the system time to start up
  time.sleep(5.0)

  while not rospy.is_shutdown():
    try:
      # Check active ROS nodes to see if 'explore' is still running
      active_nodes = rosnode.get_node_names()
      explore_running = any('explore' in node for node in active_nodes)

      # If explore was running, but has now stopped/exited
      if not explore_running:
        rospy.loginfo(
            'Explore node has finished! Saving map immediately before any'
            ' drift...'
        )

        package_path = subprocess.check_output(
            ['rospack', 'find', 'the_crop_cop']
        ).decode('utf-8').strip()
        map_path = os.path.join(package_path, 'maps', 'garden_map')

        # Save the map via map_server
        subprocess.call(['rosrun', 'map_server', 'map_saver', '-f', map_path])
        rospy.loginfo(f'Map successfully saved to {map_path}.pgm/.yaml!')

        # Kill remaining mapping nodes instantly
        os.system('rosnode kill /slam_gmapping /move_base')
        break
    except Exception as e:
      rospy.logerr(f'Error in auto-save monitor: {e}')

    time.sleep(2.0)


if __name__ == '__main__':
  try:
    check_explore_status()
  except rospy.ROSInterruptException:
    pass