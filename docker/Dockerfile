FROM ros:noetic

###### Set environment variable
ENV DISPLAY=:0
ENV LIBGL_ALWAYS_INDIRECT=0

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="UTC"

###### Install necessary packages
RUN apt-get update  && apt-get install -yy python3-pip ros-noetic-xacro
RUN pip3 install dynamixel-sdk
RUN python3 -m pip install --upgrade pip
RUN pip3 install pin
RUN pip3 install meshcat

###### Install Resources
RUN apt-get install -yy python3-catkin-tools
RUN apt-get install -yy terminator
RUN apt-get install -yy git
RUN apt-get install -yy gedit
RUN apt-get install -yy ros-noetic-rviz
RUN apt-get install -yy ros-noetic-robot-state-publisher
RUN apt-get install -yy ros-noetic-joy
RUN pip3 install pykalman

###### Create a workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src

###### Clone the necessary repositories
RUN git clone --branch devel-clean-code https://github.com/hucebot/dxl_6d_input.git
WORKDIR /root/catkin_ws/src/dxl_6d_input

###### Source the ROS setup.bash
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

###### Build the workspace
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

###### Set ROS environment variables: Tiago wired
RUN echo "export ROS_MASTER_URI=http://10.68.0.1:11311" >> ~/.bashrc
RUN echo "export ROS_IP=10.68.0.128" >> ~/.bashrc






