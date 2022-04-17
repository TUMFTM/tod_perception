# tod_lidar

This ROS package provides nodes to transmit raw and processed laser scan data from the vehicle to the operator side.
Processing on the vehicle side includes creation of a grid map and object detection as specified in the `sensors-lidar.yaml` of the vehicle config.


### Dependencies
  * C++17
  * ROS Packages: see `package.xml`
  * Other Libraries: none


### Documentation
The nodes of the package, and their functionalities are documented in the README under `src/`. 


### Quick Start
  * Follow the instructions in the container repository [teleoperated_driving](https://github.com/TUMFTM/teleoperated_driving)
  to clone and initialize the submodules.
  * Create the config of your lidar sensors, following the instructions in the
  [tod_vehicle_config](https://github.com/TUMFTM/tod_vehicle_interface/tree/master/tod_vehicle_config) package.
  * Install the `tod_lidar` dependencies and those of the required `tod_*` packages (see above).
  * Build and source the workspace.
    ```bash
    catkin build tod_lidar # with optional args to disable build of nodes on respective side: -DVEHICLE=OFF, -DOPERATOR=OFF
    source devel/setup.bash # or `setup.zsh`, depending on your shell
    ```
  * Launch the nodes of the needed side(s) using one of the following.
    ```bash
    roslaunch tod_lidar tod_lidar_vehicle.launch
    roslaunch tod_lidar tod_lidar_operator.launch
    roslaunch tod_lidar tod_lidar_both.launch
    ```
  * If not yet set, set the `/vehicleID` parameter using `rosparam set /vehicleID your-vehicle-id`.
  * For the function of the nodes, see the documentation under `src/`. 
