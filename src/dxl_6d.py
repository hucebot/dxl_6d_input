#!/usr/bin/env python3

from dynamixel_sdk import * 
import math
import time
import numpy as np
import pinocchio
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, Float32, Bool
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import PoseStamped, PointStamped

class Dxl6d:
    def __init__(self):
        rospy.init_node('dxl_input', anonymous=True)
        rospy.loginfo("dxl_input node started")

        ###### ROS parameters
        self.urdf_filename = rospy.get_param('~urdf_filename', 'src/dxl_6d_input/src/arm.urdf')
        self.ids = rospy.get_param('~ids', [1,2,3,4,5,6,7])
        self.ids = self.ids.split(',') 
        self.ids = [int(i) for i in self.ids]
        self.devicename = rospy.get_param('~devicename', '/dev/ttyUSB0')
        self.baudrate = int(rospy.get_param('~baudrate', 1000000))
        self.protocol_version = float(rospy.get_param('~protocol_version', 2.0)) 
        self.addr_present_position = int(rospy.get_param('~addr_present_position', 132)) 
        self.len_present_position = int(rospy.get_param('~len_present_position', 4))
        self.debuginfo = bool(rospy.get_param('~debuginfo', False)) 
        self.arm_side = rospy.get_param('~arm_side', 'right') 
        self.position_topic = rospy.get_param('~position_topic', '/dxl_input/pos_right') 
        self.gripper_topic = rospy.get_param('~gripper_topic', '/dxl_input/gripper_right')
        self.robot_position_topic = rospy.get_param('~robot_position_topic', '/cartesian/gripper_right_grasping_frame/current_reference')
        self.space_scalar = float(rospy.get_param('~space_scalar', 2.0))
        self.rate_ = int(rospy.get_param('~rate', 100))

        # Dynamixel torque and position addresses
        self.torque_enable_addr = 64
        self.addr_goal_position = 116
        self.initial_position = []
        self.robot_position = []
        self.first_message = True
        self.initialized = False

        ###### Pinocchio for kinematics
        self.model = pinocchio.buildModelFromUrdf(self.urdf_filename)
        self.data = self.model.createData()
        self.motor_data = [None] * len(self.ids)
        self.frame_id = self.model.getFrameId("tip")
      
        ##### Dynamixel initialization
        self.portHandler = PortHandler(self.devicename)
        self.packetHandler = PacketHandler(self.protocol_version)
        if not self.portHandler.openPort():
            rospy.logerr("Failed to open the port")
            quit() 
        if not self.portHandler.setBaudRate(self.baudrate):
            rospy.logerr("Failed to change the baudrate")
            quit()

        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.addr_present_position, self.len_present_position)
        for i in self.ids:
            self.groupSyncRead.addParam(i)

        ###### ROS publishers and subscribers
        self.pub_pos = rospy.Publisher(self.position_topic, PoseStamped, queue_size=10) 
        self.pub_gripper = rospy.Publisher(self.gripper_topic, PointStamped, queue_size=10)
        self.robot_position = rospy.wait_for_message(self.robot_position_topic, PoseStamped, timeout=5).pose.position

        self.rate = rospy.Rate(self.rate_) 
        self.pose_msg = PoseStamped()
        self.gripper_msg = PointStamped()

    # Enable torque for all motors
    def enable_torque(self):
        self.is_torque_enabled = True
        try:
            for dxl_id in self.ids:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.torque_enable_addr, 1)
                if self.debuginfo:
                    if dxl_comm_result != COMM_SUCCESS:
                        rospy.logerr(f"Failed to enable torque for ID {dxl_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                    elif dxl_error != 0:
                        rospy.logerr(f"Dynamixel error for ID {dxl_id}: {self.packetHandler.getRxPacketError(dxl_error)}")
                    else:
                        rospy.loginfo(f"Torque enabled for motor ID {dxl_id}")
        except Exception as e:
            rospy.logerr(f"Error enabling torque: {e}")

    # Disable torque for all motors
    def disable_torque(self):
        self.is_torque_enabled = False
        try:
            for dxl_id in self.ids:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.torque_enable_addr, 0)
                if self.debuginfo:
                    if dxl_comm_result != COMM_SUCCESS:
                        rospy.logerr(f"Failed to disable torque for ID {dxl_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                    elif dxl_error != 0:
                        rospy.logerr(f"Dynamixel error for ID {dxl_id}: {self.packetHandler.getRxPacketError(dxl_error)}")
                    else:
                        rospy.loginfo(f"Torque disabled for motor ID {dxl_id}")
        except Exception as e:
            rospy.logerr(f"Error disabling torque: {e}")

    # Ping a motor (for future implementation)
    def ping(self, dxl_id):
        dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, dxl_id)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr(f"Failed to ping motor ID {dxl_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            rospy.logerr(f"Dynamixel error for ID {dxl_id}: {self.packetHandler.getRxPacketError(dxl_error)}")

    # Print debug information for the robot state
    def debug(self, debuginfo):
        if debuginfo:
            rospy.loginfo(("{:<24} : {: .3f} {: .3f} {: .3f} {: .2f}"
                    .format("tip", *self.data.oMf[self.frame_id].translation.T.flat , self.gripper_msg.x)))

    # Main loop to control the robot
    def loop(self):
        while not rospy.is_shutdown():
            try:
                dxl_comm_result = self.groupSyncRead.txRxPacket()
                if dxl_comm_result != COMM_SUCCESS and self.debuginfo:
                    rospy.logerr(f'groupSyncRead txRxPacket failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}')

                # Get present position of each motor
                for i, id_ in enumerate(self.ids, start=1):
                    dxl_getdata_result = self.groupSyncRead.isAvailable(id_, self.addr_present_position, self.len_present_position)
                    if not dxl_getdata_result and self.debuginfo:
                        rospy.logerr(f"[ID:{i:03d}] groupSyncRead getdata failed")

                    present_position = self.groupSyncRead.getData(id_, self.addr_present_position, self.len_present_position)
                    self.motor_data[i - 1] = (present_position - 2048.) / 2048. * math.pi  # Convert encoder units to radians

                    # Apply inversion of direction for specific motors
                    if i == 5:
                        self.motor_data[i - 1] = -self.motor_data[i - 1]
                    
                    if i == 1 and id_ == 11:
                        self.motor_data[i - 1] -= math.pi

            except Exception as e:
                rospy.logerr(f"Error processing data from motors: {e}")

            # Forward kinematics and pose/gripper publishing (your existing code)
            q = np.array(self.motor_data[0:-1])  # Kinematic configuration excluding the gripper
            pinocchio.framesForwardKinematics(self.model, self.data, q)  # Forward kinematics
                # Get ID of the "tip" frame

            if self.initialized == False:
                self.initial_position = self.data.oMf[self.frame_id].translation.copy()
                self.initialized = True

            quat = pinocchio.Quaternion(self.data.oMf[self.frame_id].rotation)

            self.pose_msg.header.frame_id = "ci/world"
            self.pose_msg.pose.position.x = (self.data.oMf[self.frame_id].translation[0] - self.initial_position[0]) * self.space_scalar   + self.robot_position.x
            self.pose_msg.pose.position.y = (self.data.oMf[self.frame_id].translation[1] - self.initial_position[1]) * self.space_scalar  + self.robot_position.y
            self.pose_msg.pose.position.z = (self.data.oMf[self.frame_id].translation[2] - self.initial_position[2]) * self.space_scalar  + self.robot_position.z
            self.pose_msg.pose.orientation.x = quat.x
            self.pose_msg.pose.orientation.y = quat.y
            self.pose_msg.pose.orientation.z = quat.z
            self.pose_msg.pose.orientation.w = quat.w

            # Normalize the gripper data (open: -0.1135, closed: -0.3227)
            g = (self.motor_data[-1] + 0.1135) / (- 0.3227 + 0.1135)
            self.gripper_msg.point.x = 1 - np.clip(g, 0, 1)  # Clip between 0 and 1
            
            self.pose_msg.header.stamp = rospy.Time.now()
            self.gripper_msg.header = self.pose_msg.header
            self.pub_pos.publish(self.pose_msg)
            self.pub_gripper.publish(self.gripper_msg)

            self.rate.sleep()

        self.portHandler.closePort()

if __name__ == '__main__':
    node = Dxl6d()
    node.loop()
