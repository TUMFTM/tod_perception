# tod_lidar nodes
The package consists of the following nodes.

**Nodes:**
- [VehicleLaserScansSender](#vehiclelaserscanssender)
- [VehicleObjectListsSender](#vehicleobjectlistssender)
- [VehicleObjectMarkersSender](#vehicleobjectmarkerssender)
- [LaserScansDetector](#laserscansdetector)  
- [GridMapCreator](#gridmapcreator)
- [GridMapDetector](#gridmapdetector)
- [OperatorLaserScansReceiver](#operatorlaserscansreceiver)
- [OperatorObjectListsReceiver](#operatorobjectlistsreceiver)
- [OperatorObjectMarkersReceiver](#operatorobjectmarkersreceiver)


# VehicleLaserScansSender
Sends laser scan data messages for each `*` lidar device.

**Subscriptions**:
* `/Vehicle/Lidar/*/scan` 
  [sensor_msgs/LaserScan](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)


# VehicleObjectListsSender
Sends object list data messages for each `*` lidar device.

**Subscriptions**:
* `/Vehicle/Lidar/*/scan` 
  [sensor_msgs/LaserScan](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)
* `/Vehicle/Manager/status_msg` 
  [tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg)


# VehicleObjectMarkersSender
Sends object marker data messages for each `*` lidar device.

**Subscriptions**:
* `/Vehicle/Lidar/*/scan` 
  [sensor_msgs/LaserScan](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)
* `/Vehicle/Manager/status_msg`
  [tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg)


# LaserScansDetector
Uses naive Euclidean clustering to detect objects from each `*` lidar device scans and publishes them.

**Publications**:
* `/Vehicle/Lidar/*/object_list` 
  [tod_msgs/ObjectList](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/ObjectList.msg)
* `/Vehicle/Lidar/*/object_marker` 
  [visualization_msgs/Marker](http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/Marker.html)

**Subscriptions**
* `/Vehicle/Lidar/*/scan`
  [sensor_msgs/LaserScan](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)


# GridMapCreator
Creates a grid map with map being represented as a point cloud based on the device `*` data.

**Publications**:
* `/Vehicle/Lidar/GridMap/GridMapCreator/laser_scan_pointcloud`
  [sensor_msgs/PointCloud](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud.html)
* `/Vehicle/Lidar/GridMap/grid_map` 
  [grid_map_msgs/GridMap](http://docs.ros.org/en/kinetic/api/grid_map_msgs/html/msg/GridMap.html)

**Subscriptions**
* `/Vehicle/Lidar/*/scan`
  [sensor_msgs/LaserScan](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)
* `/Vehicle/VehicleBridge/odometry`
  [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)
* `/tf` [tf2_msgs/TFMessage](http://docs.ros.org/en/melodic/api/tf2_msgs/html/msg/TFMessage.html)


# GridMapDetector
Uses DBSCAN algorithm to perform clustering on grid map and publishes detected objects as object list and markers.

**Publications**:
* `/Vehicle/Lidar/GridMap/object_marker`
  [visualization_msgs/Marker](http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/Marker.html)
* `/Vehicle/Lidar/GridMap/object_list`
  [tod_msgs/ObjectList](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/ObjectList.msg)

**Subscriptions**
* `/Vehicle/Lidar/GridMap/elevation_grid`
  [nav_msgs/OccupancyGrid](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html)


# OperatorLaserScansReceiver
Receives planar laser scan data messages from `*` lidar device.

**Publications**:
 * `/Operator/Lidar/*/scan`
   [sensor_msgs/LaserScan](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)


# OperatorObjectListsReceiver
Receives list of objects from `*` lidar device.

**Publications**:
* `/Operator/Lidar/*/object_list`
  [tod_msgs/ObjectList](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/ObjectList.msg)


# OperatorObjectMarkersReceiver
Receives object markers from `*` lidar device.

**Publications**:
* `/Operator/Lidar/*/object_marker` 
  [visualization_msgs/Marker](http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/Marker.html)
