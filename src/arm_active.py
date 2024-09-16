#! /usr/bin/env python3

import os
import rospy
from std_msgs.msg import Float64MultiArray, Float32, Bool, String
from geometry_msgs.msg import PoseStamped

class SelectActiveArm:
    def __init__(self):
        rospy.init_node('select_active_arm', anonymous=True)
        rospy.loginfo("select_active_arm node started")

        self.rate = rospy.Rate(100)  # Loop rate in Hz

        self.active_arm = "right" #could be "left" or "right" or "both"

        self.gripper_position = PoseStamped()
        self.gripper = Float32()

        rospy.Subscriber('/dxl_input/active_arm', String, self.active_arm_callback)
        rospy.Subscriber('/dxl_input/pos', PoseStamped, self.position_gripper_callback)
        rospy.Subscriber('/dxl_input/gripper', Float32, self.gripper_callback)

        self.left_position_gripper_publisher = rospy.Publisher('/dxl_input/left_pos_gripper', PoseStamped, queue_size=10)
        self.right_position_gripper_publisher = rospy.Publisher('/dxl_input/right_pos_gripper', PoseStamped, queue_size=10)
        self.left_gripper_publisher = rospy.Publisher('/dxl_input/left_gripper', Float32, queue_size=10)
        self.right_gripper_publisher = rospy.Publisher('/dxl_input/right_gripper', Float32, queue_size=10)

        self.main_loop()

    def active_arm_callback(self, data):
        self.active_arm = data.data

    def position_gripper_callback(self, data):
        self.gripper_position = data

    def gripper_callback(self, data):
        self.gripper = data

    def main_loop(self):
        while not rospy.is_shutdown():
            if self.active_arm == "left":
                self.left_position_gripper_publisher.publish(self.gripper_position)
                self.left_gripper_publisher.publish(self.gripper)
            elif self.active_arm == "right":
                self.right_position_gripper_publisher.publish(self.gripper_position)
                self.right_gripper_publisher.publish(self.gripper)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        SelectActiveArm()
    except rospy.ROSInterruptException:
        rospy.loginfo("select_active_arm node terminated")