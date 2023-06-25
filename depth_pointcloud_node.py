#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from sensor_msgs import point_cloud2
import cv2

class DepthPointCloudNode:
    def __init__(self):
        rospy.init_node('depth_pointcloud_node')
        self.depth_pub = rospy.Publisher('/camera/depth', Image, queue_size=10)
        self.pointcloud_pub = rospy.Publisher('/camera/pointcloud', PointCloud2, queue_size=10)
        self.cv_bridge = CvBridge()

    def capture_depth_pointcloud(self):
        # Membaca citra kedalaman (depth image) dari sensor
        depth_image = self.read_depth_image()

        # Mengonversi citra kedalaman menjadi pesan Image ROS
        depth_msg = self.cv_bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")

        # Mengatur waktu timbang pesan citra kedalaman
        depth_msg.header.stamp = rospy.Time.now()

        # Memublikasikan pesan citra kedalaman
        self.depth_pub.publish(depth_msg)

        # Menghasilkan awan titik (point cloud) dari citra kedalaman
        pointcloud = self.generate_pointcloud(depth_image)

        # Mengatur waktu timbang pesan awan titik
        pointcloud.header.stamp = rospy.Time.now()

        # Memublikasikan pesan awan titik
        self.pointcloud_pub.publish(pointcloud)

    def read_depth_image(self):
        # Menggunakan kode OpenCV untuk membaca citra kedalaman dari sensor
        # Di sini, kita menggunakan citra kedalaman statis untuk tujuan contoh
        depth_image = cv2.imread('path_to_depth_image.png', cv2.IMREAD_ANYDEPTH)

        return depth_image

    def generate_pointcloud(self, depth_image):
        # Mengonversi citra kedalaman menjadi awan titik (point cloud)
        # Di sini, kita menggunakan titik (x, y, z) yang dihasilkan dari citra kedalaman
        points = []
        for v in range(depth_image.shape[0]):
            for u in range(depth_image.shape[1]):
                z = depth_image[v, u] / 1000.0  # Skala faktor kedalaman jika diperlukan
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points.append([x, y, z])

        fields = [point_cloud2.PointField(name='x', offset=0, datatype=7, count=1),
                  point_cloud2.PointField(name='y', offset=4, datatype=7, count=1),
                  point_cloud2.PointField(name='z', offset=8, datatype=7, count=1)]
        header = point_cloud2.create_cloud_header(frame_id='camera_frame', stamp=rospy.Time.now())
        pointcloud = point_cloud2.create_cloud_xyz32(header, points, fields)

        return pointcloud

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.capture_depth_pointcloud()
            rate.sleep()

if __name__ == '__main__':
    node = DepthPointCloudNode()
    node.run()
