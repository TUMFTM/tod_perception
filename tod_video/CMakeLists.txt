cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME tod_video)
project(${PROJECT_NAME})

set(DEPRECATION_FLAG "-Wno-deprecated-declarations -Wno-format")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEPRECATION_FLAG}")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    dynamic_reconfigure
    sensor_msgs
    geometry_msgs
    tod_msgs
    tod_network
    tod_vehicle_config
    )
find_package(Threads REQUIRED)

## find Qt 5.7
set(QT_MINIMUM_VERSION 5.7)
set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
if(Qt5_VERSION VERSION_LESS QT_MINIMUM_VERSION)
    MESSAGE(FATAL_ERROR "Minimum supported Qt version: ${QT_MINIMUM_VERSION}.
        Installed version: ${Qt5_VERSION}.")
endif()

## find paho mqtt c++
find_package(PahoMqttCpp REQUIRED)

## find GStreamer 1.0.5
set(GSTREAMER_MINIMUM_VERSION 1.0.5)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER-RTSP-SERVER gstreamer-rtsp-server-1.0)
pkg_check_modules(GSTREAMER-RTP REQUIRED gstreamer-rtp-1.0)
pkg_check_modules(GSTREAMER-APP REQUIRED gstreamer-app-1.0)


## generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
    cfg/Video.cfg
    cfg/Client.cfg
    cfg/Bitrate.cfg
    )


## catkin_package macro generates cmake config files for your package
catkin_package(
    INCLUDE_DIRS # include
    # LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp dynamic_reconfigure sensor_msgs geometry_msgs tod_msgs tod_network tod_vehicle_config
    # DEPENDS system_lib
    )


## c++ executables
set(SCENE_MANAGER_HEADER_FILES src/operator/SceneManager.h)
set(SCENE_MANAGER_SOURCE_FILES
    src/operator/SceneManagerNode.cpp
    src/operator/SceneManager.cpp
    src/operator/SceneManager.h)
qt5_wrap_cpp(SCENE_MANAGER_SOURCE_FILES ${SCENE_MANAGER_SOURCE_FILES})
qt5_wrap_ui(SCENE_MANAGER_HEADERS_MOC ui/SceneManager.ui)
add_executable(OperatorSceneManager
    ${SCENE_MANAGER_SOURCE_FILES}
    ${SCENE_MANAGER_HEADERS_MOC})
target_include_directories(OperatorSceneManager PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(OperatorSceneManager
    PRIVATE Qt5::Widgets ${catkin_LIBRARIES})
target_include_directories(OperatorSceneManager
    PRIVATE ${catkin_INCLUDE_DIRS})

add_executable(OperatorRtspClients
    src/operator/RtspClientsNode.cpp
    src/operator/RtspClients.h
    src/operator/RtspClients.cpp)
add_dependencies(OperatorRtspClients ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OperatorRtspClients
    ${catkin_LIBRARIES} ${GSTREAMER_LIBRARIES} ${GSTREAMER-RTP_LIBRARIES} ${GSTREAMER-APP_LIBRARIES} pthread)
target_include_directories(OperatorRtspClients
    PRIVATE ${catkin_INCLUDE_DIRS} ${GSTREAMER_INCLUDE_DIRS} ${GSTREAMER-RTP_INCLUDE_DIRS} ${GSTREAMER-APP_INCLUDE_DIRS})

add_executable(OperatorVideoConfigSend src/operator/VideoConfigSendNode.cpp)
add_dependencies(OperatorVideoConfigSend ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OperatorVideoConfigSend ${catkin_LIBRARIES} PahoMqttCpp::paho-mqttpp3)
target_include_directories(OperatorVideoConfigSend
    PRIVATE ${catkin_INCLUDE_DIRS})

add_executable(OperatorBitrateConfigSend src/operator/BitrateConfigSendNode.cpp)
add_dependencies(OperatorBitrateConfigSend ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OperatorBitrateConfigSend ${catkin_LIBRARIES} PahoMqttCpp::paho-mqttpp3)
target_include_directories(OperatorBitrateConfigSend
    PRIVATE ${catkin_INCLUDE_DIRS})

add_executable(OperatorBitrateIntegrator src/operator/BitrateIntegratorNode.cpp)
add_dependencies(OperatorBitrateIntegrator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OperatorBitrateIntegrator ${catkin_LIBRARIES})
target_include_directories(OperatorBitrateIntegrator
    PRIVATE ${catkin_INCLUDE_DIRS})

add_executable(VehicleConfigsReceive src/vehicle/ConfigsReceiveNode.cpp)
add_dependencies(VehicleConfigsReceive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehicleConfigsReceive ${catkin_LIBRARIES} PahoMqttCpp::paho-mqttpp3)
target_include_directories(VehicleConfigsReceive
    PRIVATE ${catkin_INCLUDE_DIRS})

add_executable(VehicleRtspServer
    src/vehicle/RtspServerNode.cpp
    src/vehicle/RtspServer.h
    src/vehicle/RtspServer.cpp)
add_dependencies(VehicleRtspServer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehicleRtspServer
    ${catkin_LIBRARIES} ${GSTREAMER_LIBRARIES} ${GSTREAMER-APP_LIBRARIES} ${GSTREAMER-RTSP-SERVER_LIBRARIES} pthread)
target_include_directories(VehicleRtspServer
    PRIVATE ${catkin_INCLUDE_DIRS} ${GSTREAMER_INCLUDE_DIRS} ${GSTREAMER-APP_INCLUDE_DIRS} ${GSTREAMER-RTSP-SERVER_INCLUDE_DIRS})

add_executable(VehicleBandwidthManager
    src/vehicle/BandwidthManagerNode.cpp
    src/vehicle/BandwidthManager.h
    src/vehicle/BandwidthManager.cpp)
add_dependencies(VehicleBandwidthManager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehicleBandwidthManager ${catkin_LIBRARIES})
target_include_directories(VehicleBandwidthManager
    PRIVATE ${catkin_INCLUDE_DIRS})

add_executable(VehiclePredQoSClient
    src/vehicle/PredQoSClientNode.cpp
    external/KDTree/KDTree.cpp
    external/KDTree/KDTree.hpp)
add_dependencies(VehiclePredQoSClient ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehiclePredQoSClient ${catkin_LIBRARIES})
target_include_directories(VehiclePredQoSClient
    PRIVATE ${catkin_INCLUDE_DIRS} external)


## install
install(TARGETS
    OperatorRtspClients
    OperatorSceneManager
    OperatorBitrateIntegrator
    OperatorVideoConfigSend
    OperatorBitrateConfigSend
    VehicleRtspServer
    VehicleBandwidthManager
    VehiclePredQoSClient
    VehicleConfigsReceive
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY
    launch
    yaml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
