docker build -f Dockerfile.amd64.intel -t chalmersrevere/opendlv-device-gpu-vulkan:v0.0.1 .

xhost +

docker run -ti --rm -e "DISPLAY=$DISPLAY" -v="/tmp/.X11-unix:/tmp/.X11-unix:rw" --privileged 3fa37ade8342 /bin/bash

./bin/opendlv-device-gpu-vulkan --cid=111

