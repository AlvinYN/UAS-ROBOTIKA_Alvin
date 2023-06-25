#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

class PointCloudProcessingNode:
    def __init__(self):
        rospy.init_node('pointcloud_processing_node')
        rospy.Subscriber('/camera/pointcloud', PointCloud2, self.pointcloud_callback)

    def pointcloud_callback(self, pointcloud_msg):
        # Mengonversi pesan awan titik menjadi daftar titik 3D
        points = []
        for point in point_cloud2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append(point)

        # Memproses daftar titik
        processed_points = self.process_pointcloud(points)

        # Menghasilkan awan titik yang diproses
        processed_pointcloud = self.generate_processed_pointcloud(processed_points, pointcloud_msg)

        # Memublikasikan pesan awan titik yang diproses
        self.publish_processed_pointcloud(processed_pointcloud)

    def process_pointcloud(self, points):
        # Implementasikan logika pemrosesan awan titik di sini
        # Di sini, kita hanya mengalikan semua koordinat titik dengan 2 untuk tujuan contoh
        processed_points = []
        for point in points:
            processed_points.append((point[0] * 2, point[1] * 2, point[2] * 2))

        return processed_points

    def generate_processed_pointcloud(self, processed_points, original_pointcloud):
        # Membuat pesan awan titik yang diproses dengan menggunakan header dari awan titik asli
        fields = original_pointcloud.fields
        header = original_pointcloud.header
        processed_pointcloud = point_cloud2.create_cloud_xyz32(header, processed_points, fields)

        return processed_pointcloud

    def publish_processed_pointcloud(self, processed_pointcloud):
        # Menerbitkan pesan awan titik yang diproses
        pub = rospy.Publisher('/camera/processed_pointcloud', PointCloud2, queue_size=10)
        processed_pointcloud.header.stamp = rospy.Time.now()
        pub.publish(processed_pointcloud)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = PointCloudProcessingNode()
    node.run()
