pkg_check_modules(GSTREAMER-RTSP-SERVER gstreamer-rtsp-server-1.0)


set(TARGETS ${TARGETS} VehicleConfigsReceive)
add_executable(VehicleConfigsReceive ConfigsReceiveNode.cpp)
add_dependencies(VehicleConfigsReceive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehicleConfigsReceive ${catkin_LIBRARIES} PahoMqttCpp::paho-mqttpp3)
target_include_directories(VehicleConfigsReceive
    PRIVATE ${catkin_INCLUDE_DIRS})


set(TARGETS ${TARGETS} VehicleRtspServer)
add_executable(VehicleRtspServer
    RtspServerNode.cpp
    RtspServer.h
    RtspServer.cpp)
add_dependencies(VehicleRtspServer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehicleRtspServer
    ${catkin_LIBRARIES} ${GSTREAMER_LIBRARIES} ${GSTREAMER-APP_LIBRARIES} ${GSTREAMER-RTSP-SERVER_LIBRARIES} pthread)
target_include_directories(VehicleRtspServer
    PRIVATE ${catkin_INCLUDE_DIRS} ${GSTREAMER_INCLUDE_DIRS} ${GSTREAMER-APP_INCLUDE_DIRS} ${GSTREAMER-RTSP-SERVER_INCLUDE_DIRS})


set(TARGETS ${TARGETS} VehicleBandwidthManager)
add_executable(VehicleBandwidthManager
    BandwidthManagerNode.cpp
    BandwidthManager.h
    BandwidthManager.cpp)
add_dependencies(VehicleBandwidthManager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehicleBandwidthManager ${catkin_LIBRARIES})
target_include_directories(VehicleBandwidthManager
    PRIVATE ${catkin_INCLUDE_DIRS})


set(TARGETS ${TARGETS} VehiclePredQoSClient)
add_executable(VehiclePredQoSClient
    PredQoSClientNode.cpp
    ../../external/KDTree/KDTree.cpp)
add_dependencies(VehiclePredQoSClient ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehiclePredQoSClient ${catkin_LIBRARIES})
target_include_directories(VehiclePredQoSClient
    PRIVATE ${catkin_INCLUDE_DIRS} ../../external)


set(TARGETS ${TARGETS} VehicleBitratePredictionSender)
add_executable(VehicleBitratePredictionSender BitratePredictionSender.cpp)
target_link_libraries(VehicleBitratePredictionSender ${catkin_LIBRARIES})
target_include_directories(VehicleBitratePredictionSender
    PRIVATE ${catkin_INCLUDE_DIRS})



install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
