#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse

head_cmd_pub = None
head_cmd_msg = JointTrajectory()

left_arm_cmd_pub = None
left_arm_cmd_msg = JointTrajectory()

right_arm_cmd_pub = None
right_arm_cmd_msg = JointTrajectory()

torso_cmd_pub = None
torso_cmd_msg = JointTrajectory()

mobile_base_cmd_pub = None
mobile_base_cmd_msg = Twist()

rate_ = 100. #10.
time_duration = 0.5 #1.0

send_commands = True

SEND_VELOCITY = False

def set_initial_configuration(data: JointState):
    home = dict()
    home["reference@v0"] = 0.0
    home["reference@v1"] = 0.0
    home["reference@v2"] = 0.0
    home["reference@v3"] = 0.0
    home["reference@v4"] = 0.0
    home["reference@v5"] = 0.0

    for i in range(len(data.name)):
        home[data.name[i]] = data.position[i]

    print("Initial configuration:")
    for key in home:
        print(f"{key}: {home[key]}")

    rospy.set_param("cartesian/home", home)

def fill_cms_msg(data:JointState, cmd_msg: JointTrajectory, rate: float, send_velocity = True):
    global time_duration
    for joint_name in cmd_msg.joint_names:
        cmd_msg.points[0].positions[cmd_msg.joint_names.index(joint_name)] = data.position[data.name.index(joint_name)]
        if send_velocity:
            cmd_msg.points[0].velocities[cmd_msg.joint_names.index(joint_name)] = data.velocity[data.name.index(joint_name)]
    cmd_msg.points[0].time_from_start = rospy.Duration(time_duration)
    return cmd_msg
def io_callback(data: JointState):
    global head_cmd_pub, head_cmd_msg
    global left_arm_cmd_pub, left_arm_cmd_msg
    global right_arm_cmd_pub, right_arm_cmd_msg
    global torso_cmd_pub, torso_cmd_msg
    global mobile_base_cmd_pub, mobile_base_cmd_msg
    global rate_, time_duration
    global send_commands

    head_cmd_msg = fill_cms_msg(data, head_cmd_msg, rate_, send_velocity=SEND_VELOCITY)
    left_arm_cmd_msg = fill_cms_msg(data, left_arm_cmd_msg, rate_, send_velocity=SEND_VELOCITY)
    right_arm_cmd_msg = fill_cms_msg(data, right_arm_cmd_msg, rate_, send_velocity=SEND_VELOCITY)
    torso_cmd_msg = fill_cms_msg(data, torso_cmd_msg, rate_, send_velocity=SEND_VELOCITY)

    mobile_base_cmd_msg.linear.x = data.velocity[0]
    mobile_base_cmd_msg.linear.y = data.velocity[1]
    mobile_base_cmd_msg.linear.z = 0.
    mobile_base_cmd_msg.angular.x = 0.
    mobile_base_cmd_msg.angular.y = 0.
    mobile_base_cmd_msg.angular.z = data.velocity[5]

    time = rospy.get_rostime()

    head_cmd_msg.header.stamp = time
    left_arm_cmd_msg.header.stamp = time
    right_arm_cmd_msg.header.stamp = time
    torso_cmd_msg.header.stamp = time

    if send_commands:
        head_cmd_pub.publish(head_cmd_msg)
        left_arm_cmd_pub.publish(left_arm_cmd_msg)
        right_arm_cmd_pub.publish(right_arm_cmd_msg)
        torso_cmd_pub.publish(torso_cmd_msg)
        mobile_base_cmd_pub.publish(mobile_base_cmd_msg)

def init_cmd_msg(cmd_msg: JointTrajectory, joint_names):
    cmd_msg.joint_names = joint_names
    cmd_msg.points.append(JointTrajectoryPoint())
    cmd_msg.points[0].positions = [0.] * len(cmd_msg.joint_names)
    cmd_msg.points[0].velocities = [0.] * len(cmd_msg.joint_names)
    return cmd_msg

def set_send_commands(req: SetBool):
    global send_commands
    send_commands = req.data
    response = SetBoolResponse()
    response.success = True
    response.message = "send_response set to " + str(req.data)
    return response

if __name__ == '__main__':
    rospy.init_node('ros_control_bridge', anonymous=True)
    print(f"SEND_VELOCITY: {SEND_VELOCITY}")

    data = rospy.wait_for_message("joint_states", JointState, timeout=5)
    set_initial_configuration(data)

    head_cmd_pub = rospy.Publisher('head_controller/command', JointTrajectory, queue_size=1)
    head_cmd_msg = init_cmd_msg(head_cmd_msg, ["head_1_joint", "head_2_joint"])

    left_arm_cmd_pub = rospy.Publisher('/arm_left_controller/command', JointTrajectory, queue_size=1)
    left_arm_cmd_msg = init_cmd_msg(left_arm_cmd_msg, ["arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint",
                        "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint"])

    right_arm_cmd_pub = rospy.Publisher('/arm_right_controller/command', JointTrajectory, queue_size=1)
    right_arm_cmd_msg = init_cmd_msg(right_arm_cmd_msg, ["arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint",
                        "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint"])

    torso_cmd_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1)
    torso_cmd_msg = init_cmd_msg(torso_cmd_msg, ["torso_lift_joint"])

    mobile_base_cmd_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)

    if rospy.has_param('~send_commands'):
        send_commands=rospy.get_param('~send_commands')
    rospy.Service('send_commands', SetBool, set_send_commands)
    print(f"send_commands: {send_commands}")

    rospy.Subscriber("cartesian/solution", JointState, io_callback)

    rate = rospy.Rate(rate_)
    while not rospy.is_shutdown():
        rate.sleep()