xhost +
docker rm cmd_client
docker rm cmd_client
docker run -it \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    --name cmd_client \
    gan_ubuntu /bin/bash -c "bash /run.sh"
xhost -
