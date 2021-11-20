cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME tod_lidar)
project(${PROJECT_NAME})

set(DEPRECATION_FLAG "-Wno-deprecated-declarations -Wno-format")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEPRECATION_FLAG}")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp tf2_ros tf2_sensor_msgs tf2_geometry_msgs sensor_msgs grid_map_ros tod_msgs tod_network tod_helper)

catkin_package(
    # INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp tf2_ros tf2_sensor_msgs tf2_geometry_msgs sensor_msgs grid_map_ros tod_msgs tod_network tod_helper
)

option(VEHICLE "Build vehicle targets" ON)
if (${VEHICLE})
    add_subdirectory(src/vehicle)
else()
    message("skipping ${PROJECT_NAME} vehicle targets")
endif()

option(OPERATOR "Build operator targets" ON)
if (${OPERATOR})
    add_subdirectory(src/operator)
else()
    message("skipping ${PROJECT_NAME} operator targets")
endif()

install(DIRECTORY
    launch
    config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

install(DIRECTORY
    external
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )
