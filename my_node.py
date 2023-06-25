#!/usr/bin/env python

import rospy

def my_node():
    # Inisialisasi node dengan nama "my_node"
    rospy.init_node('my_node', anonymous=True)

    # Loop utama node
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Tulis log pesan
        rospy.loginfo("Hello, ROS!")

        # Tunggu selama 0.1 detik
        rate.sleep()

if __name__ == '__main__':
    try:
        my_node()
    except rospy.ROSInterruptException:
        pass
