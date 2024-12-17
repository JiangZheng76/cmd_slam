xhost +
docker rm cmd_server
docker rm cmd_server
docker run -it \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    --name cmd_server \
    cmd_slam_ubuntu /bin/bash -c "/bin/bash"
xhost -
