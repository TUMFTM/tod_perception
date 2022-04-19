cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME tod_video)
project(${PROJECT_NAME})

set(DEPRECATION_FLAG "-Wno-deprecated-declarations -Wno-format")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEPRECATION_FLAG}")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    dynamic_reconfigure
    sensor_msgs
    geometry_msgs
    tod_msgs
    tod_network
    tod_helper
    tod_core

    )
find_package(Threads REQUIRED)
find_package(PahoMqttCpp REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)


generate_dynamic_reconfigure_options(
    cfg/Video.cfg
    cfg/Client.cfg
    cfg/Bitrate.cfg
    )


catkin_package(
    INCLUDE_DIRS # include
    # LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp dynamic_reconfigure sensor_msgs geometry_msgs tod_msgs tod_network tod_helper tod_core
    # DEPENDS system_lib
    )


option(OPERATOR "Build operator targets" ON)
if (${OPERATOR})
    add_subdirectory(src/operator)
else()
    message("skipping ${PROJECT_NAME} operator targets")
endif()


option(VEHICLE "Build vehicle targets" ON)
if (${VEHICLE})
    add_subdirectory(src/vehicle)
else()
    message("skipping ${PROJECT_NAME} vehicle targets")
endif()


install(DIRECTORY
    launch
    yaml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
