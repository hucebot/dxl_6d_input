import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
import math
import numpy as np
import pinocchio
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile

class Dxl6d(Node):
    def __init__(self):
        super().__init__('dxl_input')
        self.get_logger().info("dxl_input node started")

        ###### ROS parameters
        self.urdf_filename = self.declare_parameter('urdf_filename', '/ros2_ws/src/ros2_dxl_6d_input/src/arm.urdf').get_parameter_value().string_value
        self.ids = self.declare_parameter('ids', '1,2,3,4,5,6,7').get_parameter_value().string_value.split(',')
        self.ids = [int(i) for i in self.ids]
        self.devicename = self.declare_parameter('devicename', '/dev/ttyUSB0').get_parameter_value().string_value
        self.baudrate = self.declare_parameter('baudrate', 1000000).get_parameter_value().integer_value
        self.protocol_version = self.declare_parameter('protocol_version', 2.0).get_parameter_value().double_value
        self.addr_present_position = self.declare_parameter('addr_present_position', 132).get_parameter_value().integer_value
        self.len_present_position = self.declare_parameter('len_present_position', 4).get_parameter_value().integer_value
        self.debuginfo = self.declare_parameter('debuginfo', False).get_parameter_value().bool_value
        self.arm_side = self.declare_parameter('arm_side', 'right').get_parameter_value().string_value
        self.position_topic = self.declare_parameter('position_topic', '/dxl_input/pos_right').get_parameter_value().string_value
        self.gripper_topic = self.declare_parameter('gripper_topic', '/dxl_input/gripper_right').get_parameter_value().string_value
        self.robot_position_topic = self.declare_parameter('robot_position_topic', '/cartesian/gripper_right_grasping_frame/current_reference').get_parameter_value().string_value
        self.space_scalar = self.declare_parameter('space_scalar', 2.0).get_parameter_value().double_value
        self.rate_ = self.declare_parameter('rate', 100).get_parameter_value().integer_value

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
            self.get_logger().error("Failed to open the port")
            quit()
        if not self.portHandler.setBaudRate(self.baudrate):
            self.get_logger().error("Failed to change the baudrate")
            quit()

        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.addr_present_position, self.len_present_position)
        for i in self.ids:
            self.groupSyncRead.addParam(i)

        ###### ROS publishers and subscribers
        qos_profile = QoSProfile(depth=10)
        self.pub_pos = self.create_publisher(PoseStamped, self.position_topic, qos_profile)
        self.pub_gripper = self.create_publisher(Float32, self.gripper_topic, qos_profile)
        self.robot_position = self.get_robot_position()

        self.timer = self.create_timer(1.0 / self.rate_, self.loop)
        self.pose_msg = PoseStamped()
        self.gripper_msg = Float32()

    def get_robot_position(self):
        try:
            robot_position_msg = self.create_subscription(
                PoseStamped,
                self.robot_position_topic,
                lambda msg: setattr(self, 'robot_position', msg.pose.position),
                QoSProfile(depth=10)
            )
            return robot_position_msg
        except Exception as e:
            self.get_logger().error(f"Error retrieving robot position: {e}")

    def enable_torque(self):
        try:
            for dxl_id in self.ids:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.torque_enable_addr, 1)
                if self.debuginfo:
                    if dxl_comm_result != COMM_SUCCESS:
                        self.get_logger().error(f"Failed to enable torque for ID {dxl_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                    elif dxl_error != 0:
                        self.get_logger().error(f"Dynamixel error for ID {dxl_id}: {self.packetHandler.getRxPacketError(dxl_error)}")
                    else:
                        self.get_logger().info(f"Torque enabled for motor ID {dxl_id}")
        except Exception as e:
            self.get_logger().error(f"Error enabling torque: {e}")

    def loop(self):
        try:
            dxl_comm_result = self.groupSyncRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS and self.debuginfo:
                self.get_logger().error(f'groupSyncRead txRxPacket failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}')

            for i, id_ in enumerate(self.ids, start=1):
                dxl_getdata_result = self.groupSyncRead.isAvailable(id_, self.addr_present_position, self.len_present_position)
                if not dxl_getdata_result and self.debuginfo:
                    self.get_logger().error(f"[ID:{i:03d}] groupSyncRead getdata failed")

                present_position = self.groupSyncRead.getData(id_, self.addr_present_position, self.len_present_position)
                self.motor_data[i - 1] = (present_position - 2048.) / 2048. * math.pi

                if i == 5:
                    self.motor_data[i - 1] = -self.motor_data[i - 1]
                if i == 1 and id_ == 11:
                    self.motor_data[i - 1] -= math.pi

            q = np.array(self.motor_data[0:-1])
            pinocchio.framesForwardKinematics(self.model, self.data, q)

            if not self.initialized:
                self.initial_position = self.data.oMf[self.frame_id].translation.copy()
                self.initialized = True

            quat = pinocchio.Quaternion(self.data.oMf[self.frame_id].rotation)

            self.pose_msg.header.frame_id = "ci/world"
            self.pose_msg.pose.position.x = (self.data.oMf[self.frame_id].translation[0] - self.initial_position[0]) * self.space_scalar + self.robot_position.x
            self.pose_msg.pose.position.y = (self.data.oMf[self.frame_id].translation[1] - self.initial_position[1]) * self.space_scalar + self.robot_position.y
            self.pose_msg.pose.position.z = (self.data.oMf[self.frame_id].translation[2] - self.initial_position[2]) * self.space_scalar + self.robot_position.z
            self.pose_msg.pose.orientation.x = quat.x
            self.pose_msg.pose.orientation.y = quat.y
            self.pose_msg.pose.orientation.z = quat.z
            self.pose_msg.pose.orientation.w = quat.w

            g = (self.motor_data[-1] + 0.1135) / (- 0.3227 + 0.1135)
            self.gripper_msg.data = 1 - np.clip(g, 0, 1)

            self.pub_pos.publish(self.pose_msg)
            self.pub_gripper.publish(self.gripper_msg)

        except Exception as e:
            self.get_logger().error(f"Error in loop: {e}")

    def destroy_node(self):
        self.portHandler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Dxl6d()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
