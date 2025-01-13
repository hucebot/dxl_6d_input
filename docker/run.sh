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
        -v `pwd`/../:/ros2_ws/src/ros2_dxl_6d_input \
        -w /ros2_ws \
        ros2_dxl_6d:latest