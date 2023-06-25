#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

class TransformNode:
    def __init__(self):
        rospy.init_node('transform_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.transformed_point_pub = rospy.Publisher('/transformed_point', geometry_msgs.msg.PointStamped, queue_size=10)

    def transform_point(self, point):
        try:
            # Mencari transformasi dari 'base_link' ke 'map'
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time())

            # Menerapkan transformasi pada titik
            transformed_point = tf2_ros.do_transform_point(point, transform)

            # Publikasikan titik yang sudah ditransformasi
            self.transformed_point_pub.publish(transformed_point)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(str(e))

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            # Buat objek PointStamped dengan header dan koordinat yang sesuai
            point = geometry_msgs.msg.PointStamped()
            point.header.frame_id = 'base_link'
            point.header.stamp = rospy.Time.now()
            point.point.x = 1.0
            point.point.y = 0.0
            point.point.z = 0.0

            # Panggil fungsi untuk mentransformasi titik
            self.transform_point(point)

            rate.sleep()

if __name__ == '__main__':
    node = TransformNode()
    node.run()
