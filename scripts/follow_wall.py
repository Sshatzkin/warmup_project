#!/usr/bin/env python3
import rospy, numpy
from geometry_msgs.msg import Twist, Vector3
# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# How close we will get to wall in front before turning
turn_distance = 0.4

# Min and max distances from left wall
min_left_dist = 0.4
max_left_dist = 0.8

# Distance at which we will move at the max speed
max_distance = 3.0

min_turn_speed = 0.1
max_turn_speed = 0.8
min_forward_speed = 0
max_forward_speed = 0.8

# Takes an array of values and returns the smallest non-zero value
# This is used to find the nearest "object" in the ranges array for the 
#   robot to follow
def get_min_nonZero(arr):
  min_val = 10000000
  min_index = None
  for i, val in enumerate(arr):
    if val != 0.0 and val < min_val:
      min_val = val
      min_index = i
  return min_index, min_val
    
# Follower class defines the person-following behavior for the robot
class Wall_Follower:
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
    
    if (data.ranges[0] >= turn_distance): # If no wall in front...

      if (data.ranges[90] >= max_left_dist): # If measure to left is too large, turn left
        self.twist.angular.z = -0.2
      
      elif (data.ranges[90] <= min_left_dist): # If measure to left is too small, turn right
        self.twist.angular.z = 0.2
      else: # Go forward
        self.twist.angular.z = 0
        self.twist.linear.x = 0.2
    
    else: # If wall in front, turn right
      self.twist.angular.z = 0.8
      self.twist.linear.x = 0.0

    # Publish msg to cmd_vel.
    self.twist_pub.publish(self.twist)
    """if (data.ranges[0] == 0 or data.ranges[0] >= stop_distance):
      # Go forward if not facing a person / wall

      # Set the angle and distance to nearest object
      angle_to_person, dist_to_person  = get_min_nonZero(data.ranges)
      print("Angle to person: " + str(angle_to_person) + " dist: " + str(dist_to_person))
      
      # Calculate and normalize to -1:1 the error in the angle to the person
      angle_err = 0
      if angle_to_person > 180:
        angle_err = (angle_to_person - 360) / 180
      else:
        angle_err = angle_to_person / 180

      # Calculate and normalize the distance from our target distance
      dist_err = (dist_to_person - stop_distance) / (max_distance - stop_distance)

      print("Angle Err: " + str(angle_err) + "Dist err: " + str(dist_err))

      # Set Forward Speed based on distance from target and angle to target
      forward_speed = max(dist_err * max_forward_speed * (1-abs(angle_err)), min_forward_speed)
      self.twist.linear.x = forward_speed

      # Set Turn Speed based on angle to target
      turn_speed = 0
      if angle_err > 0:
        turn_speed = max(angle_err * max_turn_speed, min_turn_speed)
      else:
        turn_speed = min(angle_err * max_turn_speed, -min_turn_speed)
      self.twist.angular.z = turn_speed

      print("Forward Speed: ", forward_speed, "Turn Speed: ", turn_speed)
    else:
      # Close enough to wall, stop.
      self.twist.linear.x = 0
      self.twist.angular.z = 0"""
      
    
  
  def run(self):
    # Keep the program alive.
    rospy.spin()

if __name__ == '__main__':
  # Declare a node and run it.
  node = Follower()
  node.run()
