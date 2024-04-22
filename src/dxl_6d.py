#!/usr/bin/env python3
from dynamixel_sdk import * 
import rospy
from std_msgs.msg import Float64MultiArray

PROTOCOL_VERSION            = 2.0
DEVICENAME                  = '/dev/ttyUSB0'
BAUDRATE                    = 1000000
ADDR_PRESENT_POSITION       = 132
LEN_PRESENT_POSITION        = 4         # Data Byte Length
IDS                         = [1,2,3,4,5,6]



def dxl_node():
    # ROS part
    pub = rospy.Publisher('pos', Float64MultiArray, queue_size=10)
    rospy.init_node('dxl_input', anonymous=True)
    rate = rospy.Rate(500) # 500hz


    # DXL part
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    if not portHandler.openPort():
        print("Failed to open the port")
        quit()

    if not portHandler.setBaudRate(BAUDRATE):  
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        quit()


    # Initialize GroupSyncRead instace for Present Position
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

    # Add parameter storage for Dynamixel#1 present position value
    for i in IDS:
        groupSyncRead.addParam(i)

    msg = Float64MultiArray()
    data = [None] * len(IDS)
    while not rospy.is_shutdown():
        # Syncread present position
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Check if groupsyncread data of Dynamixel#1 is available
        for i in IDS:
            dxl_getdata_result = groupSyncRead.isAvailable(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % i)
            # Get Dynamixel#1 present position value
            present_position = groupSyncRead.getData(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            #print(i, "=>", present_position * 360 / 4096)
            #print(i, len(data))
            data[i - 1] = present_position * 360. / 4096
            #print(present_position * 360. / 4096)

        msg.data = data
        pub.publish(msg)
        rate.sleep()

    # Close port
    portHandler.closePort()


if __name__ == '__main__':
    try:
        dxl_node()
    except rospy.ROSInterruptException:
        pass