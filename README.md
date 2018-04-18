# opendlv-device-gpu-vulkan

Compile with (replace 'intel' with 'nvidia' or 'amd' depending on your platform):

```
docker build -f Dockerfile.amd64.intel -t chalmersrevere/opendlv-device-gpu-vulkan:v0.0.1 .
```

Run the newely created docker image by:

```
xhost+
docker run -ti --rm -e "DISPLAY=$DISPLAY" -v="/tmp/.X11-unix:/tmp/.X11-unix:rw" --privileged <docker-image-id> /bin/bash
./bin/opendlv-device-gpu-vulkan --cid=111
```

## License
This project is released under the terms of the GNU GPLv3 License - [![License: GPLv3](https://img.shields.io/badge/license-GPL--3-blue.svg
)](https://www.gnu.org/licenses/gpl-3.0.txt)
