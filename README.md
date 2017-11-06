# opendlv.cfsd18

## Building using a Docker builder:

    cd docker
    make buildComplete
    make createDockerImage

## Run the resulting Docker image:

    docker run -ti --rm --net host --user odv chalmersrevere/opendlv-cfsd18-lynx-on-opendlv-on-opendlv-core-on-opendavinci-on-base:latest /bin/bash

