#!/usr/bin/env python3

import os
import subprocess
import time
import rosnode
import rospy
from nav_msgs.msg import OccupancyGrid


class CombinedAutoSave:

  def __init__(self):
    rospy.init_node('auto_save_on_finish', anonymous=True)

    # Fallback timer (300s default) + smart early-save triggers
    self.fallback_delay_sec = float(
        rospy.get_param('~save_delay_sec', 300.0)
    )
    self.startup_grace_sec = float(rospy.get_param('~startup_grace_sec', 10.0))
    self.plateau_sec = float(rospy.get_param('~plateau_sec', 120.0))
    self.min_growth_cells = int(rospy.get_param('~min_growth_cells', 25))

    self.start_monotonic = time.monotonic()
    self.explore_was_seen = False

    self.last_explored = -1
    self.last_growth_monotonic = None
    self.map_saved = False

    self.sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

    remaining = max(0, int(round(self.fallback_delay_sec)))
    rospy.logwarn(
        '[AutoSave] Active: watching explore/map plateau with %ds fallback'
        ' timer.',
        remaining,
    )

  def map_callback(self, msg):
    if self.map_saved or not msg.data:
      return

    now = time.monotonic()
    explored_cells = sum(1 for cell in msg.data if cell != -1)
    if explored_cells == 0:
      return

    if self.last_growth_monotonic is None:
      self.last_explored = explored_cells
      self.last_growth_monotonic = now
    elif abs(explored_cells - self.last_explored) >= self.min_growth_cells:
      self.last_explored = explored_cells
      self.last_growth_monotonic = now

  def check_explore_finished(self, elapsed):
    if elapsed < self.startup_grace_sec:
      return False

    try:
      active_nodes = rosnode.get_node_names()
      explore_running = any('explore' in node for node in active_nodes)
      if explore_running:
        self.explore_was_seen = True
      elif self.explore_was_seen:
        rospy.logwarn(
            '[AutoSave] Explore node has finished! Saving map immediately...'
        )
        return True
    except Exception:
      pass
    return False

  def check_map_plateau(self, elapsed):
    if elapsed < self.startup_grace_sec or self.last_growth_monotonic is None:
      return False
    plateau_elapsed = time.monotonic() - self.last_growth_monotonic
    if plateau_elapsed >= self.plateau_sec:
      rospy.logwarn(
          '[AutoSave] Map growth plateaued for %.1fs. Saving map...',
          plateau_elapsed,
      )
      return True
    return False

  def trigger_save_and_cleanup(self, reason):
    if self.map_saved:
      return
    self.map_saved = True

    try:
      package_path = (
          subprocess.check_output(['rospack', 'find', 'the_crop_cop'])
          .decode('utf-8')
          .strip()
      )
      maps_dir = os.path.join(package_path, 'maps')
      os.makedirs(maps_dir, exist_ok=True)
      map_path = os.path.join(maps_dir, 'garden_map')

      rospy.logwarn(
          '[AutoSave] (%s) Saving map to %s.pgm/.yaml...', reason, map_path
      )
      ret = subprocess.call(
          ['rosrun', 'map_server', 'map_saver', '-f', map_path]
      )
      rospy.logwarn('[AutoSave] map_saver exited with code %d', ret)

      subprocess.call(
          ['rosnode', 'kill', '/explore', '/slam_gmapping', '/move_base'],
          stdout=subprocess.DEVNULL,
          stderr=subprocess.DEVNULL,
      )
      subprocess.call(
          ['pkill', '-f', 'rviz'],
          stdout=subprocess.DEVNULL,
          stderr=subprocess.DEVNULL,
      )
    except Exception as e:
      rospy.logerr('[AutoSave] Error saving map: %s', e)

    rospy.signal_shutdown('Auto-save complete.')

  def run(self):
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown() and not self.map_saved:
      elapsed = time.monotonic() - self.start_monotonic
      remaining = max(0, int(round(self.fallback_delay_sec - elapsed)))

      if self.check_explore_finished(elapsed):
        self.trigger_save_and_cleanup('explore exited')
        break

      if self.check_map_plateau(elapsed):
        self.trigger_save_and_cleanup('map plateau')
        break

      if elapsed >= self.fallback_delay_sec:
        rospy.logwarn('[AutoSave] Saving map in 0...')
        rospy.logwarn('[AutoSave] Time up! Fallback timer reached.')
        self.trigger_save_and_cleanup('300s fallback timer')
        break

      rospy.logwarn(
          '[AutoSave] Fallback save in %ds (or earlier on explore finish /'
          ' plateau)...',
          remaining,
      )
      rate.sleep()


if __name__ == '__main__':
  try:
    node = CombinedAutoSave()
    node.run()
  except rospy.ROSInterruptException:
    pass