FROM ros:humble

###### Set environment variable
ENV DISPLAY=:0
ENV LIBGL_ALWAYS_INDIRECT=0
ENV ROS_DISTRO=humble

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="UTC"

###### Install necessary packages
RUN apt-get update  && apt-get install -yy python3-pip ros-${ROS_DISTRO}-xacro
RUN pip3 install dynamixel-sdk
RUN python3 -m pip install --upgrade pip
RUN pip3 install pin
RUN pip3 install meshcat

###### Install Resources
RUN apt-get install -yy terminator
RUN apt-get install -yy git
RUN apt-get install -yy gedit
RUN apt-get install -yy ros-${ROS_DISTRO}-rviz2

###### Source the ROS setup.bash
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

###### Create a workspace
RUN mkdir -p /ros2_ws/src

WORKDIR /ros2_ws/src
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"



