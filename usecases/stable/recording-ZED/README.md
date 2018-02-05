This folder provides the instructions for recording with proxy-camera, which can be executed from a docker-compose file to start all micro-services to record video streams from three OpenCV cameras with lossless H264 compression. It is assumed that git, Docker, and Docker Compose are installed and all the three cameras are properly connected. To install Docker, follow the tutorial: https://docs.docker.com/engine/installation/linux/ubuntulinux/
    
### Prepare proxy-camera

proxy-camera is included in the opendlv.core repository (https://github.com/chalmers-revere/opendlv.core). Clone the opendlv.core source:

    $ git clone https://github.com/chalmers-revere/opendlv.core
    
    $ git pull
    
Go to opendlv.core/docker, build and create the Docker image seresearch/opendlv-core-on-opendavinci-on-base:latest:

    $ make buildComplete
    
    $ make createDockerImage
    
### Use proxy-camera with Docker Compose

Go to the folder usecases/recording.cameras.opencv.3. This folder contains a configuration file, a docker-compose file docker-compose.yml, and an environment file .env. The environment file .env defines an environment variable CID which is referred to by the docker-compose file. CID is a user-defined environment variable that specifies the cid of the UDP session established by odsupercomponent. In .env CID has the value 111, thus in docker-compose.yml "${CID}" resolves to 111.  Run Docker Compose:
    
    $ docker-compose up --build

Then proxy-camera will start the recording with all the three cameras. To stop the recording, run

    $ docker-compose stop
    
Then remove all stopped containers:

    $ docker-compose rm

After the recording, the recording files are stored at ~/recordings, including three .h264 files as the actual recordings with lossless H264 compression. The recording file format is CID-xxx-odrecorderh264_yyy, where xxx is the cid number and yyy is the timestamp.

Note that the value of CID defined in .env can be manually overwritten by preceding the docker-compose command with CID=xxx, where xxx is the cid number. For instance, the following command makes odsupercomponent, proxy-camera, and odrecorderh264 run with cid 123 instead of 111:

    $ CID=123 docker-compose up
    
Then CID=123 should also be used for docker-compose stop and docker-compose rm accordingly.

