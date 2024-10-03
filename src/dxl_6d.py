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
        self.position_topic = rospy.get_param('~position_topic', '/dxl_input/pos_right')  # Topic for the current reference position
        self.gripper_topic = rospy.get_param('~gripper_topic', '/dxl_input/gripper_right')  # Topic for the gripper state
        self.robot_position_topic = rospy.get_param('~robot_position_topic', '/cartesian/gripper_right_grasping_frame/current_reference')  # Topic for the robot position
        self.space_scalar = float(rospy.get_param('~space_scalar', 2.0)) # Scalar for the workspace
        self.using_pedal = bool(rospy.get_param('~using_pedal', True))  # Enable/disable pedal control
        self.using_streamdeck = bool(rospy.get_param('~using_streamdeck', False))  # Enable/disable streamdeck control

        # Dynamixel torque and position addresses
        self.torque_enable_addr = 64
        self.addr_goal_position = 116
        self.initial_position = []
        self.robot_position = []
        self.first_message = True
        self.initialized = False
        self.teleoperation_mode = True
        self.is_torque_enabled = False
        self.data = [None] * len(self.ids)

        ###### Pinocchio for kinematics
        self.model = pinocchio.buildModelFromUrdf(self.urdf_filename)
        self.data = self.model.createData()
      
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
        self.pub_gripper = rospy.Publisher(self.gripper_topic, Float32, queue_size=10)
        self.robot_state_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.robot_position = rospy.wait_for_message(self.robot_position_topic, PoseStamped, timeout=5).pose.position

        if self.using_streamdeck:
            rospy.Subscriber('/hucebot_streamdeck/teleoperation_mode', Bool, self.teleoperation_mode_callback)

        if self.using_pedal:
            rospy.Subscriber('/joy', Joy, self.send_command_robot)
            self.send_command = False
            self.enable_torque()
        else:
            self.send_command = True
            self.disable_torque()

        self.rate = rospy.Rate(20) 
        self.pose_msg = PoseStamped()
        self.gripper_msg = Float32()

    def teleoperation_mode_callback(self, msg):
        self.teleoperation_mode = msg.data

    def send_command_robot(self, msg):
        if msg.axes[0] >= 0.8 and msg.axes[1] >= 0.8 or msg.axes[0] >= 0.8 and msg.axes[1] >= 0.8:
            self.send_command = False
            if self.is_torque_enabled == False:
                self.enable_torque()

        elif msg.axes[0] <= -0.8:
            self.send_command = True
            if self.is_torque_enabled == True:
                self.disable_torque()

        elif msg.axes[1] <= -0.8 and msg.axes[0] >= 0.8 or msg.axes[1] <= -0.8 and msg.axes[0] == 0:
            self.send_command = False
            if self.is_torque_enabled == True:
                self.disable_torque()
            self.robot_position = rospy.wait_for_message(self.robot_position_topic, PoseStamped, timeout=5).pose.position
            self.initialized = False

        

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
                    .format("tip", *self.data.oMf[frame_id].translation.T.flat , self.gripper_msg.data)))

    # Main loop to control the robot
    def loop(self):
        joint_state_msg = JointState()
        joint_state_msg.name = [f'arm_joint_{i}' for i in range(1, len(self.ids)+1)] 
        joint_state_msg.velocity = []
        joint_state_msg.effort = []
        
        initialized = False
        while not rospy.is_shutdown():
            if self.teleoperation_mode:
                try:
                    dxl_comm_result = self.groupSyncRead.txRxPacket()
                    if dxl_comm_result != COMM_SUCCESS:
                        rospy.logerr(f'groupSyncRead txRxPacket failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}')

                    # Get present position of each motor
                    for i, id_ in enumerate(self.ids, start=1):
                        dxl_getdata_result = self.groupSyncRead.isAvailable(id_, self.addr_present_position, self.len_present_position)
                        if not dxl_getdata_result:
                            rospy.logerr(f"[ID:{i:03d}] groupSyncRead getdata failed")

                        present_position = self.groupSyncRead.getData(id_, self.addr_present_position, self.len_present_position)
                        self.data[i - 1] = (present_position - 2048.) / 2048. * math.pi  # Convert encoder units to radians

                        # Apply inversion of direction for specific motors
                        if i == 5:
                            self.data[i - 1] = -data[i - 1]
                        
                        if i == 1 and id_ == 11:
                            self.data[i - 1] -= math.pi

                except Exception as e:
                    rospy.logerr(f"Error processing data from motors: {e}")
                
                # Update joint state message with current joint positions
                joint_state_msg.position = self.data 
                joint_state_msg.header.stamp = rospy.Time.now()  # Add timestamp

                # Publish the joint states
                self.robot_state_publisher.publish(joint_state_msg)

                # Forward kinematics and pose/gripper publishing (your existing code)
                q = np.array(self.data[0:-1])  # Kinematic configuration excluding the gripper
                pinocchio.framesForwardKinematics(self.model, self.data, q)  # Forward kinematics
                frame_id = self.model.getFrameId("tip")  # Get ID of the "tip" frame

                if self.initialized == False:
                    self.initial_position = self.data.oMf[frame_id].translation.copy()
                    self.initialized = True

                quat = pinocchio.Quaternion(self.data.oMf[frame_id].rotation)

                self.pose_msg.header.frame_id = "ci/world"
                self.pose_msg.pose.position.x = (self.data.oMf[frame_id].translation[0] - self.initial_position[0]) * self.space_scalar   + self.robot_position.x
                self.pose_msg.pose.position.y = (self.data.oMf[frame_id].translation[1] - self.initial_position[1]) * self.space_scalar  + self.robot_position.y
                self.pose_msg.pose.position.z = (self.data.oMf[frame_id].translation[2] - self.initial_position[2]) * self.space_scalar  + self.robot_position.z
                self.pose_msg.pose.orientation.x = quat.x
                self.pose_msg.pose.orientation.y = quat.y
                self.pose_msg.pose.orientation.z = quat.z
                self.pose_msg.pose.orientation.w = quat.w

                # Normalize the gripper data (open: -0.1135, closed: -0.3227)
                g = (data[-1] + 0.1135) / (- 0.3227 + 0.1135)
                self.gripper_msg.data = 1 - np.clip(g, 0, 1)  # Clip between 0 and 1
                
                # Publish the pose and gripper data
                if self.send_command:
                    self.pub_pos.publish(self.pose_msg)
                    self.pub_gripper.publish(self.gripper_msg)

                self.rate.sleep()

        self.portHandler.closePort()

if __name__ == '__main__':
    node = Dxl6d()
    node.loop()
