#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node')
        self.image_pub = rospy.Publisher('/camera/image', Image, queue_size=10)
        self.cv_bridge = CvBridge()

    def capture_image(self):
        # Membaca gambar dari kamera
        image = self.read_camera()

        # Mengonversi gambar OpenCV menjadi pesan Image ROS
        image_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")

        # Mengatur waktu timbang pesan gambar
        image_msg.header.stamp = rospy.Time.now()

        # Memublikasikan pesan gambar
        self.image_pub.publish(image_msg)

    def read_camera(self):
        # Menggunakan kode OpenCV untuk membaca gambar dari kamera
        # Di sini, kita menggunakan gambar statis untuk tujuan contoh
        image = cv2.imread('path_to_image.jpg')

        return image

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.capture_image()
            rate.sleep()

if __name__ == '__main__':
    node = CameraNode()
    node.run()
