# tod_video
This ROS package provides a low-latency video streaming framework. It handles configuration, control and transmission of multiple videos from the teleoperated vehicle to the operator. There are three different video rate control modes (Single, Automatic, Collective). More details can be found in the documentation and the publication, linked below.


### Dependencies
  * ROS Packages:
    * roscpp
    * dynamic_reconfigure
    * sensor_msgs
    * geometry_msgs
    * tod_msgs
    * tod_network
    * tod_vehicle_config
  * Third Party:
    * [GStreamer](https://gstreamer.freedesktop.org/) (gstreamer-1.0, gstreamer-rtsp-server-1.0, gstreamer-rtp-1.0, gstreamer-app-1.0)
      ```
      sudo apt-get install libgstreamer1.0-0 -y
      sudo apt-get install gstreamer1.0-plugins-base -y
      sudo apt-get install gstreamer1.0-plugins-good -y
      sudo apt-get install gstreamer1.0-plugins-bad -y
      sudo apt-get install gstreamer1.0-plugins-ugly -y
      sudo apt-get install gstreamer1.0-libav -y
      sudo apt-get install gstreamer1.0-doc -y
      sudo apt-get install gstreamer1.0-tools -y
      sudo apt-get install gstreamer1.0-x -y
      sudo apt-get install gstreamer1.0-alsa -y
      sudo apt-get install gstreamer1.0-gl -y
      sudo apt-get install gstreamer1.0-gtk3 -y
      sudo apt-get install gstreamer1.0-qt5 -y
      sudo apt-get install gstreamer1.0-pulseaudio -y
      sudo apt-get install libgstreamer1.0-dev -y
      sudo apt-get install libgstreamer-plugins-base1.0-dev -y
      sudo apt-get install libgstrtspserver-1.0-dev -y
      ```
    * [PahoMqttCpp](https://github.com/eclipse/paho.mqtt.cpp)  
    Install instructions can be found in the [tod_network](https://github.com/TUMFTM/tod_common/tree/master/tod_network) package.
    * [Qt5](https://www.qt.io/) (Widgets Core)
      ```
      sudo apt-get install qt5-default qt5-qmake qtbase5-dev-tools qt5-doc -y
      ```


### Documentation
More details on the modes and underlying concepts of the video streaming framework can be found in the publication. 

The nodes of the package and their functionalities are documented in the README under [src/](https://github.com/TUMFTM/tod_perception/tree/master/tod_video/src).


### Quick Start
  * Follow the instructions in the container repository [teleoperated_driving](https://github.com/TUMFTM/teleoperated_driving/tree/master)
  to clone and initialize the submodules.
  * Create the config of your camera system, following the instructions in the
  [tod_vehicle_config](https://github.com/TUMFTM/tod_vehicle_interface/tree/master/tod_vehicle_config) package.
  * Install the `tod_video` dependencies and those of the required `tod_*` packages (see above). 
  * Build and source the workspace.
    ```
    catkin build tod_video && source devel/setup.bash # or `setup.zsh`, depending on your shell
    ```
  * On the vehicle side: 
    * Set your `vehicleID` in the `tod_video_vehicle.launch` and launch the vehicle nodes.
      ```
      roslaunch tod_video tod_video_vehicle.launch
      ```
    * The GStreamer pipelines of the `RtspServer` will be initialized and ready to be connected to once the first image of the respective camera is received,
    and the `tod_status` in the status message, published by Vehicle Manager node, changes to something different then `IDLE`.
    * To simulate a connection from the operator, execute the following command, publishing the status message, in a separate terminal.
      ```
      rostopic pub -r 10 /Vehicle/Manager/status_msg tod_msgs/StatusMsg '{tod_status: 1}'
      ```
  * On the operator side: 
    * Set your `vehicleID` in the `tod_video_operator.launch` and launch the operator nodes.
      ```
      roslaunch tod_video tod_video_operator.launch
      ```
    * The `RtspClients` will establish the connection to the server once the `tod_status` in the status message, published by Operator Manager node,
    changes to something different then `IDLE`.
    * To trigger a connection to the server at the `vehicle_vehicle_ip_address`, execute the following command, publishing the status message, in a separate terminal.
      ```
      rostopic pub -r 10 /Operator/Manager/status_msg tod_msgs/StatusMsg {'vehicle_vehicle_ip_address: '127.0.0.1', tod_status: 1}'
      ```
    * The image output format, published by the `RtspClients`, can be set with the parameter `imageOutputFormat` (default=`RGB8`) in the launch file.
  * Adaptation of video parameters is disabled by default (parameter `adaptVideoParamsEnabled` in launch files). 
  Therewith, certain functionality of the scene manager GUI is disabled, leaving only the option to play/pause videos using the tick boxes.
  * To make full use of the video streaming framework (set ip addresses, connect via gui, adapt video stream parameters, switch between the video rate control modes, ...),
  the `tod_manager` package is required. 

*Note (Feb. 11, 2021):* The `tod_manager` package will be published in the coming weeks.


### Publication
*todo*

