from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('ros2_dxl_6d_input'),
        'config',
        'dxl_config.yaml'
    )

    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)

    return LaunchDescription(
        generate_nodes(config)
    )

def generate_nodes(config_file):
    # General parameters
    protocol_version = config_file['general']['protocol_version']
    baudrate = config_file['general']['baudrate']
    addr_present_position = config_file['general']['addr_present_position']
    len_present_position = config_file['general']['len_present_position']
    urdf_filename = config_file['general']['urdf_filename']
    debuginfo = config_file['general']['debuginfo']
    space_scalar = config_file['general']['space_scalar']
    rate = config_file['general']['rate']

    # Left arm parameters
    devicename_left = config_file['left_arm']['devicename']
    ids_left = config_file['left_arm']['ids']
    arm_side_left = config_file['left_arm']['arm_side']
    left_position_topic = config_file['left_arm']['position_topic']
    left_gripper_topic = config_file['left_arm']['gripper_topic']
    left_robot_position_topic = config_file['left_arm']['robot_position_topic']
    using_left_arm = config_file['left_arm']['enabled']

    # Right arm parameters
    devicename_right = config_file['right_arm']['devicename']
    ids_right = config_file['right_arm']['ids']
    arm_side_right = config_file['right_arm']['arm_side']
    right_position_topic = config_file['right_arm']['position_topic']
    right_gripper_topic = config_file['right_arm']['gripper_topic']
    right_robot_position_topic = config_file['right_arm']['robot_position_topic']
    using_right_arm = config_file['right_arm']['enabled']

    node_list = []

    # Left arm node
    if using_left_arm:
        left_arm_node = Node(
            package='ros2_dxl_6d_input',
            executable='dxl_6d',
            name='left_dxl_6d_input',
            output='screen',
            parameters=[
                {
                    'protocol_version': protocol_version,
                    'devicename': devicename_left,
                    'baudrate': baudrate,
                    'addr_present_position': addr_present_position,
                    'len_present_position': len_present_position,
                    'ids': ids_left,
                    'urdf_filename': urdf_filename,
                    'debuginfo': debuginfo,
                    'arm_side': arm_side_left,
                    'position_topic': left_position_topic,
                    'gripper_topic': left_gripper_topic,
                    'robot_position_topic': left_robot_position_topic,
                    'space_scalar': space_scalar,
                    'rate': rate
                }
            ]
        )
        node_list.append(left_arm_node)

    # Right arm node
    if using_right_arm:
        right_arm_node = Node(
            package='ros2_dxl_6d_input',
            executable='dxl_6d',
            name='right_dxl_6d_input',
            output='screen',
            parameters=[
                {
                    'protocol_version': protocol_version,
                    'devicename': devicename_right,
                    'baudrate': baudrate,
                    'addr_present_position': addr_present_position,
                    'len_present_position': len_present_position,
                    'ids': ids_right,
                    'urdf_filename': urdf_filename,
                    'debuginfo': debuginfo,
                    'arm_side': arm_side_right,
                    'position_topic': right_position_topic,
                    'gripper_topic': right_gripper_topic,
                    'robot_position_topic': right_robot_position_topic,
                    'space_scalar': space_scalar,
                    'rate': rate
                }
            ]
        )
        node_list.append(right_arm_node)

    return node_list
