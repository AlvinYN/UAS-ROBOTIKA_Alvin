#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from my_robot_msgs.srv import AddTwoInts

class ExampleNode:
    def __init__(self):
        rospy.init_node('example_node')
        rospy.Subscriber('input_topic', String, self.callback)
        self.result = 0
        self.add_two_ints_service = rospy.Service('add_two_ints', AddTwoInts, self.handle_add_two_ints)

    def callback(self, msg):
        rospy.loginfo("Received message: %s", msg.data)

    def handle_add_two_ints(self, req):
        self.result = req.a + req.b
        rospy.loginfo("Adding %d and %d", req.a, req.b)
        return self.result

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo("Result: %d", self.result)
            rate.sleep()

if __name__ == '__main__':
    node = ExampleNode()
    node.run()
