#!/usr/bin/env python3
import rospy, numpy
from geometry_msgs.msg import Twist, Vector3
# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# How close we will get to wall.
stop_distance = 0.4

class Follower:
  def __init__(self):
    # Start rospy node.
    rospy.init_node("walk_to_wall")
    # Declare our node as a subscriber to the scan topic and
    #   set self.process_scan as the function to be used for callback.
    rospy.Subscriber("/scan", LaserScan, self.process_scan)
    # Get a publisher to the cmd_vel topic.
    self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    # Create a default twist msg (all values 0).
    lin = Vector3()
    ang = Vector3()
    self.twist = Twist(linear=lin,angular=ang)

  def process_scan(self, data):
    # Get the number of points in the scan.
    num_points = len(data.ranges)

    if (data.ranges[0] == 0.0 or data.ranges[0] >= stop_distance):
      # Go forward if not close enough to wall.
      self.twist.linear.x = 0.1
    else:
      # Close enough to wall, stop.
      self.twist.linear.x = 0

    # Publish msg to cmd_vel.
    self.twist_pub.publish(self.twist)
  
  def run(self):
    # Keep the program alive.
    rospy.spin()

if __name__ == '__main__':
  # Declare a node and run it.
  node = Follower()
  node.run()