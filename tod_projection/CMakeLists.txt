cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME tod_projection)
project(${PROJECT_NAME})

set(DEPRECATION_FLAG "-Wno-deprecated-declarations -Wno-format")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEPRECATION_FLAG}")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp sensor_msgs nav_msgs tf2_ros tod_msgs tod_helper tod_core
    )


catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs nav_msgs tf2_ros tod_msgs tod_helper
    )


option(OPERATOR "Build operator targets" ON)
if (${OPERATOR})
    add_subdirectory(src)
else()
    message("skipping ${PROJECT_NAME} operator targets")
endif()


install(DIRECTORY 
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
