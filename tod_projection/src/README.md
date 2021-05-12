# tod_projection nodes
The package consists of the following nodes.

**Nodes:**
- [OperatorVideoProjection](#operatorvideoprojection)
- [OperatorLaneProjection](#operatorlaneprojection)


# OperatorVideoProjection
If connected to the vehicle, publishes textures with the projections of the received lane and laser scan data.

**Advertised Services:**
- `/Operator/Projection/*/image_raw`
  ([sensor_msgs/Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))
  Texture of the projections of each camera '*'.

**Subscribed Topics:**
- `/Operator/Video/*/image_raw` ([sensor_msgs/Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))
  Raw images of each camera '*'.
- `/Operator/Projection/vehicle_lane_front_*` ([nav_msgs/Path](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Path.html))
  Lane of the vehicle's front edges ('*': left/right) in the `base_footprint` frame.
- `/Operator/Lidar/*/scan` ([sensor_msgs/LaserScan](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html))
  Laser scan of of each 2D lidar '*' on the vehicle.
- `/Operator/Manager/status_msg` ([tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg))
  Connection status to operator.
- `/tf` ([tf2_msgs/TFMessage](http://docs.ros.org/en/melodic/api/tf2_msgs/html/msg/TFMessage.html)) Transforms from the `/base_footprint` frame to the camera frames, and from the laser scan frames to the camera frames.


# OperatorLaneProjection
Publishes the set of vehicle lanes (front left/right, rear left/right) on every receive of a vehicle data message.

**Advertised Services:**
- `/Operator/Projection/vehicle_lane_*_**` ([nav_msgs/Path](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Path.html))
  Lane of the vehicle edge (\*: front/rear, \*\*: left/right).

**Subscribed Topics:**
- `/Operator/VehicleBridge/vehicle_data`
  ([tod_msgs/VehicleData](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/VehicleData.msg))
  Contains current steering wheel angle of the vehicle that is used to calculate the lanes. 
