#!/usr/bin/env python3
import rospy, numpy, math
from geometry_msgs.msg import Twist, Vector3
# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# How close we will get to wall in front before turning
turn_distance = 0.3

# Min and max distances from left wall
min_left_dist = 0.3
max_left_dist = 0.4

adjustment_speed = 0.2
right_turn_speed = -0.5
forward_speed = 0.3
  
# Returns 0 if alligned, 1 for left, -1 for right
def alligned(ranges):
  if ((ranges[90] > max_left_dist or ranges[90] == 0.0) or (ranges[75] > max_left_dist or ranges[75] == 0.0) or (ranges[60] > max_left_dist * (2/math.sqrt(3)) or ranges[60] == 0.0)):
     if (ranges[90] == 0.0):
       return 1
     else:
      return ranges[90] / 3.5 
  elif ((ranges[90] < min_left_dist and ranges[90] != 0.0) or (ranges[75] < min_left_dist and ranges[75] != 0.0) or (ranges[60] < min_left_dist * (2/math.sqrt(3)) and ranges[60] != 0.0)):
      return -1 * (ranges[90] / 3.5) 
  else:
      return 0
  
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
    if (data.ranges[270] < 0.4 and data.ranges[270] != 0.0):
      print("Stopping")
      self.twist.angular.z = 0
      self.twist.linear.x = 0 
    elif (data.ranges[0] >= turn_distance or data.ranges[0] == 0.0): # If no wall in front...
      alignment = alligned(data.ranges)
      print("Alignment: ", alignment)
      if (alignment >= 0): # If measure to left is too large, turn left
        self.twist.angular.z = 0.2
        self.twist.linear.x = 0.2
        if (data.ranges[45] >= max_left_dist * 2 or data.ranges[45] == 0.0):
          self.twist.angular.z = 0.5
          self.twist.linear.x = 0.1
        print("Too far form left, turning left")
      elif (alignment < 0): # If measure to left is too small, turn right
        self.twist.angular.z = -0.2
        self.twist.linear.x = 0.2
        print("Too close to left, turning right")
      else: # Go forward
        self.twist.angular.z = 0
        self.twist.linear.x = forward_speed
    
    else: # If wall in front, turn right
      print("Wall in front, turning right")
      self.twist.angular.z = right_turn_speed
      self.twist.linear.x = -0.1

      rospy.sleep(0.4)

    # Publish msg to cmd_vel.
    self.twist_pub.publish(self.twist)
  
  def run(self):
    # Keep the program alive.
    rospy.spin()

if __name__ == '__main__':
  # Declare a node and run it.
  node = Wall_Follower()
  node.run()
