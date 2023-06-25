#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class SensingVisualizationNode:
    def __init__(self):
        rospy.init_node('sensing_visualization_node')
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    def scan_callback(self, scan):
        ranges = scan.ranges
        min_range = min(ranges)

        # Publish marker jika jarak terdekat lebih kecil dari 1.0 meter
        if min_range < 1.0:
            marker = Marker()
            marker.header.frame_id = scan.header.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = min_range
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            self.marker_pub.publish(marker)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = SensingVisualizationNode()
    node.run()
