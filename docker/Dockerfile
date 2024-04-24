FROM ros:noetic

RUN apt-get update  && apt-get install -yy python3-pip
RUN pip3 install dynamixel-sdk
RUN python3 -m pip install --upgrade pip
RUN pip3 install pin
RUN pip3 install meshcat

