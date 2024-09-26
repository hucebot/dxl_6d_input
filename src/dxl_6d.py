#!/usr/bin/env python3

from dynamixel_sdk import * 
import math
import pinocchio
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, Float32, Bool
from geometry_msgs.msg import PoseStamped

# Class to handle communication with a 6-DOF Dynamixel robot
class Dxl6d:
    def __init__(self):
        rospy.init_node('dxl_input', anonymous=True)
        rospy.loginfo("dxl_input node started")

        ###### ROS parameters (can be set through launch files or parameter server)
        self.urdf_filename = rospy.get_param('~urdf_filename', 'src/dxl_6d_input/src/arm.urdf')  # Path to the URDF file
        self.ids = rospy.get_param('~ids', [1,2,3,4,5,6,7])  # Dynamixel motor IDs
        self.ids = self.ids.split(',')  # Split the string into a list
        self.ids = [int(i) for i in self.ids]  # Convert to integers
        self.devicename = rospy.get_param('~devicename', '/dev/ttyUSB0')  # Device name for serial communication
        self.baudrate = int(rospy.get_param('~baudrate', 1000000))  # Baudrate for communication
        self.protocol_version = float(rospy.get_param('~protocol_version', 2.0))  # Dynamixel protocol version
        self.addr_present_position = int(rospy.get_param('~addr_present_position', 132))  # Address for present position
        self.len_present_position = int(rospy.get_param('~len_present_position', 4))  # Length of the position data
        self.debuginfo = bool(rospy.get_param('~debuginfo', False))  # Enable/disable debug info
        self.arm_side = rospy.get_param('~arm_side', 'right')  # Side of the arm (right or left)

        # Dynamixel torque and position addresses
        self.torque_enable_addr = 64
        self.addr_goal_position = 116

        ###### Pinocchio for kinematics
        self.model = pinocchio.buildModelFromUrdf(self.urdf_filename)  # Load robot model from URDF
        self.data = self.model.createData()  # Create data for kinematic computations
      
        ##### Dynamixel initialization
        self.portHandler = PortHandler(self.devicename)
        self.packetHandler = PacketHandler(self.protocol_version)
        if not self.portHandler.openPort():  # Open the communication port
            rospy.logerr("Failed to open the port")
            quit()  # Exit on error
        if not self.portHandler.setBaudRate(self.baudrate):  # Set the baudrate
            rospy.logerr("Failed to change the baudrate")
            quit()

        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.addr_present_position, self.len_present_position)
        for i in self.ids:
            self.groupSyncRead.addParam(i)  # Add motors to synchronous read

        self.disable_torque()  # Disable torque for all motors

        ###### ROS publishers and subscribers
        pos_topic = f'/dxl_input/pos_{self.arm_side}'  # Topic for position data
        gripper_topic = f'/dxl_input/gripper_{self.arm_side}'  # Topic for gripper control
        self.pub_pos = rospy.Publisher(pos_topic, PoseStamped, queue_size=10)  # Publisher for position
        self.pub_gripper = rospy.Publisher(gripper_topic, Float32, queue_size=10)  # Publisher for gripper status

        self.rate = rospy.Rate(100)  # Loop rate in Hz
        self.pose_msg = PoseStamped()  # Pose message to publish
        self.gripper_msg = Float32()  # Gripper message to publish

    # Enable torque for all motors
    def enable_torque(self):
        for dxl_id in self.ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.torque_enable_addr, 1)
            if self.debuginfo:  # Print debug information if enabled
                if dxl_comm_result != COMM_SUCCESS:
                    rospy.logerr(f"Failed to enable torque for ID {dxl_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                elif dxl_error != 0:
                    rospy.logerr(f"Dynamixel error for ID {dxl_id}: {self.packetHandler.getRxPacketError(dxl_error)}")
                else:
                    rospy.loginfo(f"Torque enabled for motor ID {dxl_id}")

    # Disable torque for all motors
    def disable_torque(self):
        for dxl_id in self.ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.torque_enable_addr, 0)
            if self.debuginfo:
                if dxl_comm_result != COMM_SUCCESS:
                    rospy.logerr(f"Failed to disable torque for ID {dxl_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                elif dxl_error != 0:
                    rospy.logerr(f"Dynamixel error for ID {dxl_id}: {self.packetHandler.getRxPacketError(dxl_error)}")
                else:
                    rospy.loginfo(f"Torque disabled for motor ID {dxl_id}")

    # Move all motors to the initial position
    def go_to_initial_position(self):
        initial_position = [44, 2560, 2560]
        for motor_id, motor_position in enumerate(initial_position):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id + 1, self.torque_enable_addr, 1)
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, motor_id + 1, self.addr_goal_position, motor_position)
            if self.debuginfo:
                if dxl_comm_result != COMM_SUCCESS:
                    rospy.logerr(f"Failed to move motor ID {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                elif dxl_error != 0:
                    rospy.logerr(f"Dynamixel error for ID {motor_id}: {self.packetHandler.getRxPacketError(dxl_error)}")
                else:
                    rospy.loginfo(f"Motor ID {motor_id} moved to initial position")

            rospy.sleep(0.3)  # Wait for the motor to reach the position

    # Ping a motor (for future implementation)
    def ping(self, dxl_id):
        pass

    # Print debug information for the robot state
    def debug(self, debuginfo):
        if debuginfo:
            rospy.loginfo(("{:<24} : {: .3f} {: .3f} {: .3f} {: .2f}"
                    .format("tip", *self.data.oMf[frame_id].translation.T.flat , self.gripper_msg.data)))

    # Main loop to control the robot
    def loop(self):
        data = [None] * len(self.ids)
        while not rospy.is_shutdown():
            dxl_comm_result = self.groupSyncRead.txRxPacket()  # Read data from all motors
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

            # Get present position of each motor
            for i, id_ in enumerate(self.ids, start=1):
                dxl_getdata_result = self.groupSyncRead.isAvailable(id_, self.addr_present_position, self.len_present_position)
                if not dxl_getdata_result:
                    rospy.logerr(f"[ID:{i:03d}] groupSyncRead getdata failed")

                present_position = self.groupSyncRead.getData(id_, self.addr_present_position, self.len_present_position)
                data[i - 1] = (present_position - 2048.) / 2048. * math.pi  # Convert encoder units to radians
                if i == 2 or i == 3 or i == 4:
                    data[i-1] = -data[i-1]  # Invert direction for specific motors

            q = np.array(data[0:-1])  # Kinematic configuration excluding the gripper
            pinocchio.framesForwardKinematics(self.model, self.data, q)  # Forward kinematics
            frame_id = self.model.getFrameId("tip")  # Get ID of the "tip" frame

            # Fill pose message with position and orientation data
            quat = pinocchio.Quaternion(self.data.oMf[frame_id].rotation)
            self.pose_msg.header.frame_id = "map"
            self.pose_msg.pose.position.x = self.data.oMf[frame_id].translation[0] * 2.0
            self.pose_msg.pose.position.y = self.data.oMf[frame_id].translation[1] * 2.0
            self.pose_msg.pose.position.z = self.data.oMf[frame_id].translation[2] * 2.0
            self.pose_msg.pose.orientation.x = quat.x
            self.pose_msg.pose.orientation.y = quat.y
            self.pose_msg.pose.orientation.z = quat.z
            self.pose_msg.pose.orientation.w = quat.w

            # Normalize the gripper data (open: -0.1135, closed: -0.3227)
            g = (data[-1] + 0.1135) / (- 0.3227 + 0.1135)
            self.gripper_msg.data = np.clip(g, 0, 1)  # Clip between 0 and 1
            
            # Publish the pose and gripper data
            self.pub_pos.publish(self.pose_msg)
            self.pub_gripper.publish(self.gripper_msg)
            self.rate.sleep()

        self.portHandler.closePort()

if __name__ == '__main__':
    node = Dxl6d()
    node.loop()
