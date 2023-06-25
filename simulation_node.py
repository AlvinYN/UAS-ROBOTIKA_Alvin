#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SimulationNode:
    def __init__(self):
        rospy.init_node('simulation_node')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def scan_callback(self, scan):
        ranges = scan.ranges
        min_range = min(ranges)
        rospy.loginfo("Minimum range: %.2f", min_range)

        if min_range > 1.0:
            self.move_forward()
        else:
            self.stop()

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = SimulationNode()
    node.run()
