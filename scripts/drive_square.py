#!/usr/bin/env python3

from turtle import forward
import rospy

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

class DriveSquare(object):
    """ This node publishes ROS messages containing the 3D coordinates of a single point """

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('drive_square', anonymous=True)
        # setup publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def run(self):
        # setup the Twist message we want to send
        forward = Twist(
            linear=Vector3(0.3, 0, 0),
            angular=Vector3(0, 0, 0)
        )

        left_turn = Twist(
            linear=Vector3(0, 0, 0),
            angular=Vector3(0, 0, 0.6)
        )

        stop = Twist(
            linear=Vector3(0, 0, 0),
            angular=Vector3(0, 0, 0)
        )
        rospy.sleep(1)
        # Drive in a square
        for i in range(4):
            self.robot_movement_pub.publish(forward)
            rospy.sleep(4)
            self.robot_movement_pub.publish(left_turn)
            rospy.sleep(3)
        self.robot_movement_pub.publish(stop)

if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = DriveSquare()
    node.run()
