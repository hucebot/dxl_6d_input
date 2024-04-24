#!/usr/bin/env python3
from dynamixel_sdk import * 
import math
import pinocchio
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped

# these should be ros parameters, for future work
PROTOCOL_VERSION            = 2.0
DEVICENAME                  = '/dev/ttyUSB0'
BAUDRATE                    = 1000000
ADDR_PRESENT_POSITION       = 132
LEN_PRESENT_POSITION        = 4         # Data Byte Length
IDS                         = [1,2,3,4,5,6]
urdf_filename = 'src/dxl_6d_input/src/arm.urdf'



class Dxl6d:
    def __init__(self):
        ###### pinocchio
        self.model = pinocchio.buildModelFromUrdf(urdf_filename)
        self.data = self.model.createData()
      
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
        self.pub = rospy.Publisher('pos', PoseStamped, queue_size=10)
        rospy.init_node('dxl_input', anonymous=True)
        self.rate = rospy.Rate(100) # 100hz
        self.pose_msg = PoseStamped()

    def loop(self):
        msg = Float64MultiArray()
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
                if i == 2 or i == 4 or i == 5:
                    data[i-1] = -data[i-1]
                print(present_position, data[i-1] / math.pi * 180)
            q = np.array(data)
            pinocchio.forwardKinematics(self.model, self.data, q)

            # Print out the placement of each joint of the kinematic tree
            for name, oMi in zip(self.model.names, self.data.oMi):
                print(("{:<24} : {: .3f} {: .3f} {: .3f}"
                    .format( name, *oMi.translation.T.flat )))

            msg.data = data
            quat = pinocchio.Quaternion(self.data.oMi[5].rotation)
            self.pose_msg.header.frame_id = "map"
            self.pose_msg.pose.position.x = self.data.oMi[5].translation[0]
            self.pose_msg.pose.position.y = self.data.oMi[5].translation[1]
            self.pose_msg.pose.position.z = self.data.oMi[5].translation[2]
            self.pose_msg.pose.orientation.x = quat.x
            self.pose_msg.pose.orientation.y = quat.y
            self.pose_msg.pose.orientation.z = quat.z
            self.pose_msg.pose.orientation.w = quat.w
            
            print(self.pose_msg)
            self.pub.publish(self.pose_msg)
            self.rate.sleep()

        # Close port
        self.portHandler.closePort()


if __name__ == '__main__':
    node = Dxl6d()
    node.loop()
