This folder provides the instructions for using proxy-velodyne16, a program which decodes a live stream from a VLP-16 lidar. This use case is particularly used for adjusting the mounting position and angle of a VLP-16 lidar on the car. A docker-compose file is provided to start all micro-services to decode VLP-16 packets and visualize them as 3D point cloud. It includes three services: odsupercomponent, odcockpit, and opendlv-core-system-proxy-velodyne16 (or proxy-velodyne16 for short). odsupercomponent is used for software component lifecycle management in OpenDaVINCI. odcockpit is a visualization tool of OpenDaVINCI. proxy-velodyne16 listens to VLP-16 packets and decodes them in real time. This tutorial assumes that git, Docker, and Docker Compose are installed. To install Docker, follow the tutorial: https://docs.docker.com/engine/installation/linux/ubuntulinux/. In addition, the Docker image of opendlv.core is necessary for running this use case.

VLP-16 sends out data as UDP packets via an Ethernet cable. Currently VLP-16 is configured to have a static IP address via DHCP: 10.42.42.150. Ping this IP address to check if VLP-16 is connected to the network. Run `nmap 10.42.42.0/24` if necessary to check the IP addresses of all devices in the network. The network configuration of VLP-16 can be changed by following the data sheet provided together with VLP-16.

The VLP-16 decoder supports two types of point clouds: shared point cloud (SPC) and compact point cloud (CPC). SPC contains the xyz coordinate of each point and its intensity value. CPC is a compressed version of SPC, squeezing one VLP-16 frame into one UDP packet. CPC does not include intensity values and it is less accurate than SPC, however, the disk consumption of CPC is much lower. In the configuration file in this use case folder, proxy-velodyne16.pointCloudOption specifies which point cloud is enabled: 0: SPC only, 1: CPC only, 2: both SPC and CPC. This use case folder also contains:

- a Dockerfile specifying the Docker image to be used
- an environment file .env which defines an environment variable CID that is referred to by the docker-compose file
- a VLP-16 calibration file VLP-16.xml required by the VLP-16 decoder while SPC is enabled
- a car model file Car.objx and a simulation scenario file Track.scnx. They are not directly useful to this use case, however, these two files are required by the EnvironmentViewer plugin of odcockpit to visualize the point cloud

Here CID is a user-defined environment variable that specifies the cid of the UDP session established by odsupercomponent. In .env CID has the value 111, thus in docker-compose.yml "${CID}" resolves to 111. In this folder, run Docker Compose (the first command grants access to your Xserver):

    $ xhost +
    
    $ docker-compose up --build

This will activate odsupercomponent, the visualization tool odcockpit, and proxy-velodyne16. The VLP-16 packets will be visualized as 3D point cloud in the EnvironmentViewer plugin in odcockpit. In EnvironmentViewer, unselect the stationary elements XYZAxes, Grid, Surroundings, AerialImage and the dynamic element EgoCar to have a clean background for the point cloud. By default, EnvironmentViewer uses free camera view which allows a user to do the following operations:

- Use **W**/**S** on the keyboard to zoom in and zoom out
- Use **A**/**D** on the keyboard to move the display window left and right
- Drag the vertical bar on the left to adjust the perspective (the same operation can also be performed in the display window with the same effect)
- Drag the horizontal bar at the bottom to rotate clockwise and counter-clockwise (the same operation can also be performed in the display window with the same effect)

To stop proxy-velodyne16, run

    $ docker-compose stop
    
Remove the stopped containers:

    $ docker-compose rm
    
Note that the value of CID defined in .env can be manually overwritten by preceding the docker-compose command with CID=xxx, where xxx is the cid number. For instance, the following command makes all micro-services run with cid 123 instead of 111:

    $ CID=123 docker-compose up --build

Then CID=123 should also be used for docker-compose stop and docker-compose rm accordingly.

