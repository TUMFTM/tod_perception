cmake_minimum_required(VERSION 2.8.3)

set(TARGETS ${TARGETS} VehicleLaserScansSender)
add_executable(VehicleLaserScansSender VehicleLaserScansSender.cpp)
target_link_libraries(VehicleLaserScansSender ${catkin_LIBRARIES})
target_include_directories(VehicleLaserScansSender PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET VehicleLaserScansSender PROPERTY CXX_STANDARD 17)

set(TARGETS ${TARGETS} VehicleObjectListsSender)
add_executable(VehicleObjectListsSender VehicleObjectListsSender.cpp)
target_link_libraries(VehicleObjectListsSender ${catkin_LIBRARIES})
target_include_directories(VehicleObjectListsSender PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET VehicleObjectListsSender PROPERTY CXX_STANDARD 17)

set(TARGETS ${TARGETS} VehicleObjectMarkersSender)
add_executable(VehicleObjectMarkersSender VehicleObjectMarkersSender.cpp)
target_link_libraries(VehicleObjectMarkersSender ${catkin_LIBRARIES})
target_include_directories(VehicleObjectMarkersSender PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET VehicleObjectMarkersSender PROPERTY CXX_STANDARD 17)

set(TARGETS ${TARGETS} LaserScansDetector)
add_executable(LaserScansDetector LaserScansDetector.cpp LaserScansDetectorNode.cpp
		../../include/tod_lidar/ObjectListHelper.h)
target_link_libraries(LaserScansDetector ${catkin_LIBRARIES})
target_include_directories(LaserScansDetector PUBLIC ${catkin_INCLUDE_DIRS} ../../include/)
set_property(TARGET LaserScansDetector PROPERTY CXX_STANDARD 17)

set(TARGETS ${TARGETS} GridMapCreator)
add_executable(GridMapCreator GridMapCreator.cpp GridMapCreatorNode.cpp)
target_link_libraries(GridMapCreator ${catkin_LIBRARIES})
target_include_directories(GridMapCreator PUBLIC ${catkin_INCLUDE_DIRS})
set_property(TARGET GridMapCreator PROPERTY CXX_STANDARD 17)

set(TARGETS ${TARGETS} GridMapDetector)
add_executable(GridMapDetector GridMapDetector.cpp GridMapDetectorNode.cpp
		../../include/tod_lidar/ObjectListHelper.h
                ../../external/SimpleDBSCAN/kdtree.cpp)
target_link_libraries(GridMapDetector ${catkin_LIBRARIES})
target_include_directories(GridMapDetector PUBLIC ${catkin_INCLUDE_DIRS} ../../include ../../external)
set_property(TARGET GridMapDetector PROPERTY CXX_STANDARD 17)

install(TARGETS ${TARGETS}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
