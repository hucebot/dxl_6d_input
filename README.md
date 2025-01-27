# dxl_6d_input

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](
https://opensource.org/licenses/BSD-3-Clause)
[![Ros Version](https://img.shields.io/badge/ROS-Noetic-green)](
http://wiki.ros.org/noetic)

ROS package for a 6-d input device based on dynamixels.

The end effector of the device is a 6-DOF manipulator with 2 MX-64 and 5 MX-28 Dynamixels. The device is controlled by a ROS node that reads the position of the end effector and publishes it to a topic as also the position of the gripper. The node also subscribes to a topic to receive the actual position of the robot end effector to compensate the initial position of the device.

### Topics
#### Publishers
- `/dxl_6d_input/pos_{side}` - Publishes the position of the end effector.
- `/dxl_6d_input/gripper_{side}` - Publishes the position of the gripper.

#### Subscribers
- `/cartesian/gripper_{side}_grasping_frame/current_reference` - Subscribes to the position of the robot end effector.
- `/streamdeck/teleoperation_mode` - Subscribes to the teleoperation mode of the device.
- `/joy` - Subscribes to the joystick input.

### Parameters
- `~urdf_filename` - The URDF file of the device.
- `~ids` - The IDs of the Dynamixels.
- `~devicename` - The name of the port where the USB2Dynamixel is connected.
- `~baudrate` - The baudrate of the USB2Dynamixel.
- `~protocol_version` - The protocol version of the USB2Dynamixel.
- `~addr_present_position` - The address of the present position register.
- `~len_present_position` - The length of the present position register.
- `~debuginfo` - Flag to enable the debug information.
- `~arm_side` - The side of the arm.
- `~position_topic` - The topic where the position of the robot end effector is published.
- `~gripper_topic` - The topic where the position of the gripper is published.
- `~robot_position_topic` - The topic where the position of the robot end effector is subscribed.
- `~space_scalar` - The scalar to convert the position of the robot end effector to the device.

# Get Started

| 3D Design                               | 3D Design                               |
|---------------------------------------------------|---------------------------------------------------|
| <img src="https://github.com/hucebot/dxl_6d_input/blob/bimanual-teleoperation/images/3d_design1.png" alt="3D Design"> | <img src="https://github.com/hucebot/dxl_6d_input/blob/bimanual-teleoperation/images/3d_design2.png" alt="3D Design"> |


## Hardware
For the hardware setup, the package is designed to work with the 2 Dynamixel MX-64 and 5 Dynamixel MX-28 actuators. All the 3D parts can be found in the `dxl_6d_input/armd_design` folder where you will find the FreeCad files and the STL files for 3D printing. To assemble the device, you can follow the next steps:

### Parts - By Side
- 2 Dynamixel MX-64.
- 5 Dynamixel MX-28.
- Dynamixel cables.
- 1 USB2Dynamixel.
- 3D printed parts.
- Screws and nuts (M2, M3, M4, M5).
- 1 aluminum profile (10x10 mm) - 100 mm.

### Assembly
- Print the parts in the `dxl_6d_input/armd_design` folder. All the parts are designed to be printed with a **0.4mm nozzle and 0.2mm** layer height, with **40% infill** and **supports enabled**.
    - The files `finger_1.stl`, `finger_2.stl`, `griper_axis.stl`, `griper_axis_1.stl`, `griper_axis_1.stl` should be mirrored to print the left arm.
- The 2 Dynamixel MX-64 should be mounted on the base of the device, while the 5 Dynamixel MX-28 should be mounted on the arm.
- We strongly recommend to assemble from the bottom to the top.

## Installation

### From Docker

The easiest way to get started is to use the provided Docker image. You can find the Dockerfile in the `docker` folder. To build the image, run the following command:

```docker build -t dxl_6d_input .```

The run the command below to start the container (inside the `docker` folder):

```sh run_docker.sh```

# Usage

## Prepare the Dynamixels

Before starting the node, make sure that the Dynamixels are connected to the computer and that the USB2Dynamixel is connected to the computer. Also check the following:
- The protocol version is set to 2.0.
- The baudrate is set to 1Mbps.
- The left arm it's connected to /dev/ttyUSB0 and the right arm to /dev/ttyUSB1.
- The Dynamixels are connected in the following order:

<center>

| Left Arm | Right Arm  |
| ------------- | ------------- |
| MX-64 - ID 1  | MX-64 - ID 11  |
| MX-64 - ID 2  | MX-64 - ID 12  |
| MX-28 - ID 3  | MX-28 - ID 13  |
| MX-28 - ID 4  | MX-28 - ID 14  |
| MX-28 - ID 5  | MX-28 - ID 15  |
| MX-28 - ID 6  | MX-28 - ID 16  |
| MX-28 - ID 7  | MX-28 - ID 17  |

</center>

## Start the Node

Once the Dynamixels are connected, you can start the node running the following command:

```roslaunch dxl_6d_input dxl_6d_input.launch```
