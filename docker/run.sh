# For graphics
xhost +local:docker

docker run \
        -it \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=0" \
        --net host \
        --privileged \
        -v /dev:/dev \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -v $HOME/ros_ws:/root/catkin_ws \
        dxl_6d:latest