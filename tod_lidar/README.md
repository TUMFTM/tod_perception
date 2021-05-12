# tod_lidar

This ROS package provides nodes to
  * transmit the raw and processed data to the operator
  * process laser scan data (grid map, object detection) on the vehicle side


### Dependencies
  * ROS Packages:
    * roscpp
    * tf2_ros
    * sensor_msgs
    * grid_map_ros
    * tod_msgs
    * tod_network
    * tod_vehicle_config (exec only)
    * grid_map_visualization (exec only)


### Documentation
The nodes of the package, and their functionalities are documented in the README under `src/`. 


### Quick Start
  * Follow the instructions in the container repository [teleoperated_driving](https://github.com/TUMFTM/teleoperated_driving)
  to clone and initialize the submodules.
  * Create the config of your lidar sensors, following the instructions in the
  [tod_vehicle_config](https://github.com/TUMFTM/tod_vehicle_interface/tree/master/tod_vehicle_config) package.
  * Install the `tod_lidar` dependencies and those of the required `tod_*` packages (see above).
  * Build and source the workspace.
    ```
    catkin build tod_lidar # with optional args to disable build of nodes on respective side: -DVEHICLE=OFF, -DOPERATOR=OFF
    source devel/setup.bash # or `setup.zsh`, depending on your shell
    ```
  * Set your `vehicleID` and the respective side (vehicle or operator) in the `tod_lidar.launch` and launch the nodes.
    ```
    roslaunch tod_lidar tod_lidar.launch
    ```
  * For the function of the nodes, see the documentation under `src/`. 
