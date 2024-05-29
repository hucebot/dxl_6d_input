#!/usr/bin/env python3
from dynamixel_sdk import * 
import math
import pinocchio
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import PoseStamped

# these should be ros parameters, for future work
PROTOCOL_VERSION            = 2.0
DEVICENAME                  = '/dev/ttyUSB0'
BAUDRATE                    = 1000000
ADDR_PRESENT_POSITION       = 132
LEN_PRESENT_POSITION        = 4         # Data Byte Length
IDS                         = [1,2,3,4,5,6,7]
urdf_filename = 'src/dxl_6d_input/src/arm.urdf'



class Dxl6d:
    def __init__(self):
        ###### pinocchio
        self.model = pinocchio.buildModelFromUrdf(urdf_filename)
        self.data = self.model.createData() # pinocchio data object
      
        ##### dynamixel
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        if not self.portHandler.openPort():
            print("Failed to open the port")
            quit() # TODO exception
        if not self.portHandler.setBaudRate(BAUDRATE):  
            print("Failed to change the baudrate")
            quit()
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        for i in IDS:
            self.groupSyncRead.addParam(i)

        ###### ROS
        self.pub_pos = rospy.Publisher('/dxl_input/pos', PoseStamped, queue_size=10)
        self.pub_gripper = rospy.Publisher('/dxl_input/gripper', Float32, queue_size=10)
        rospy.init_node('dxl_input', anonymous=True)
        self.rate = rospy.Rate(100) # 100hz
        self.pose_msg = PoseStamped()
        self.gripper_msg = Float32()

        # check the available IDS
    
    def ping(self, dxl_id):
        pass

    def loop(self):
        data = [None] * len(IDS)
        while not rospy.is_shutdown():
            dxl_comm_result = self.groupSyncRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

            for i in IDS:
                dxl_getdata_result = self.groupSyncRead.isAvailable(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                if dxl_getdata_result != True:
                    print("[ID:%03d] groupSyncRead getdata failed" % i)
                # Get Dynamixel#1 present position value
                present_position = self.groupSyncRead.getData(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                #print(i, "=>", present_position * 360 / 4096)
                #print(i, len(data))
                data[i - 1] = (present_position - 2048.) / 2048. * math.pi
                if i == 2 or i == 5:
                    data[i-1] = -data[i-1]
                if i == 6:
                    data[i-1] -= math.pi    # modification by enrico mingo
                print(present_position, data[i-1] / math.pi * 180)
            q = np.array(data[0:-1])
            pinocchio.framesForwardKinematics(self.model, self.data, q) # we ignore the gripper in the kinematics
            frame_id = self.model.getFrameId("tip")
            print(("{:<24} : {: .3f} {: .3f} {: .3f}"
                    .format("tip", *self.data.oMf[frame_id].translation.T.flat )))
            # for name, oMi in zip(self.model.names, self.data.oMi):
            #     print(("{:<24} : {: .3f} {: .3f} {: .3f}"
            #         .format( name, *oMi.translation.T.flat )))

            quat = pinocchio.Quaternion(self.data.oMf[frame_id].rotation)
            self.pose_msg.header.frame_id = "map"
            self.pose_msg.pose.position.x = self.data.oMf[frame_id].translation[0] * 2.0
            self.pose_msg.pose.position.y = self.data.oMf[frame_id].translation[1] * 2.0
            self.pose_msg.pose.position.z = self.data.oMf[frame_id].translation[2] * 2.0
            self.pose_msg.pose.orientation.x = quat.x
            self.pose_msg.pose.orientation.y = quat.y
            self.pose_msg.pose.orientation.z = quat.z
            self.pose_msg.pose.orientation.w = quat.w



            # gripper: fully open = -1.2, fully closed = -2.05
            # -> normalize so that 0 = closed, 1 = open
            g = (data[-1] + 1.2) / (- 2.05 + 1.2)
            self.gripper_msg.data = np.clip(g, 0, 1)

            print(self.pose_msg, self.gripper_msg)

            self.pub_pos.publish(self.pose_msg)
            self.pub_gripper.publish(self.gripper_msg)
            self.rate.sleep()

        # Close port
        self.portHandler.closePort()


if __name__ == '__main__':
    node = Dxl6d()
    node.loop()
