This folder provides the instructions for visualizing Compact Point Cloud (CPC) from VLP-16 sent as UDP packets from another module in the same network. This use case starts odcockpit whcih is a visualization tool of OpenDaVINCI. While this use case is running, it allows the remote monitoring of CPC if another use case SendCPC (next to this use case) is running with the same CID on the same or another computer in the same network. It is assumed that Docker and and Docker Compose are installed. To install Docker, follow the tutorial: https://docs.docker.com/engine/installation/linux/ubuntulinux/. In addition, the Docker image of opendlv.core is necessary for running this use case.

### Replay video recordings
This folder contains:

-a docker-compose file that includes odcockpit as the micro-service
- a Dockerfile specifying the Docker image to be used
- an environment file .env which defines an environment variable CID that is referred to by the docker-compose file
- a car model file Car.objx and a simulation scenario file Track.scnx. They are not directly useful to this use case, however, these two files are required by the EnvironmentViewer plugin of odcockpit to visualize the point cloud

CID is a user-defined environment variable that specifies the cid of the UDP session established by odsupercomponent. In .env CID has the value 111, thus in docker-compose.yml "${CID}" resolves to 111.  To start odcockpit, run Docker Compose (the first command grants access to your Xserver):

    $ xhost +
    
    $ docker-compose up --build

Then incoming CPC will be visualized in the EnvironmentViewer plugin in odcockpit. In EnvironmentViewer, unselect the stationary elements XYZAxes, Grid, Surroundings, AerialImage and the dynamic element EgoCar to have a clean background for the point cloud. By default, EnvironmentViewer uses free camera view which allows a user to do the following operations:

- Use **W**/**S** on the keyboard to zoom in and zoom out
- Use **A**/**D** on the keyboard to move the display window left and right
- Drag the vertical bar on the left to adjust the perspective (the same operation can also be performed in the display window with the same effect)
- Drag the horizontal bar at the bottom to rotate clockwise and counter-clockwise (the same operation can also be performed in the display window with the same effect)

To stop odcockpit, run

    $ docker-compose stop
    
Then remove all stopped containers:

    $ docker-compose rm

Note that the value of CID defined in .env can be manually overwritten by preceding the docker-compose command with CID=xxx, where xxx is the cid number. For instance, the following command makes odcockpit run with cid 123 instead of 111:

    $ CID=123 docker-compose up --build

Then CID=123 should also be used for docker-compose stop and docker-compose rm accordingly.
