#!/usr/bin/env python3
import rospy, numpy
from geometry_msgs.msg import Twist, Vector3
# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# How close we will get to wall.
stop_distance = 0.4
max_turn_speed = 1
max_forward_speed = 0.8
def get_min_nonZero(arr):
  min_val = 10000000
  min_index = None
  for i, val in enumerate(arr):
    if val != 0.0 and val < min_val:
      min_val = val
      min_index = i
  return min_index, min_val
    
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
    
    if (data.ranges[0] == 0 or data.ranges[0] >= stop_distance):
      # Go forward if not close enough to wall.
      print("Going Forward - range: " + str(data.ranges[0]))

      
      angle_to_person, dist_to_person  = get_min_nonZero(data.ranges)

      # Set Forward Speed
      self.twist.linear.x = (dist_to_person / 3.5) * max_forward_speed
      #if (angle_to_person > 330 or angle_to_person < 30):
      #  self.twist.linear.x = (dist_to_person / 3.5) * max_forward_speed
      #else:
      #  self.twist.linear.x = 0.1
      print("Angle to closest obj: ", angle_to_person)

      
      # Set Turn Speed
      turn_speed = 0
      if (angle_to_person > 180):
        turn_speed = ((angle_to_person - 360) / 180) * max_turn_speed
      else:
        turn_speed = (angle_to_person / 180) * max_turn_speed
     
      print("Turn Speed: ", turn_speed)

      self.twist.angular.z = turn_speed
    else:
      # Close enough to wall, stop.
      self.twist.linear.x = 0
      self.twist.angular.z=0
    # Publish msg to cmd_vel.
    self.twist_pub.publish(self.twist)
  
  def run(self):
    # Keep the program alive.
    rospy.spin()

if __name__ == '__main__':
  # Declare a node and run it.
  node = Follower()
  node.run()
