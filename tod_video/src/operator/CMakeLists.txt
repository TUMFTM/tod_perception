set(QT_MINIMUM_VERSION 5.7)
set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
if(Qt5_VERSION VERSION_LESS QT_MINIMUM_VERSION)
    MESSAGE(FATAL_ERROR "Minimum supported Qt version: ${QT_MINIMUM_VERSION}.
        Installed version: ${Qt5_VERSION}.")
endif()

set(SCENE_MANAGER_HEADER_FILES SceneManager.h)
set(SCENE_MANAGER_SOURCE_FILES
    SceneManagerNode.cpp
    SceneManager.cpp
    SceneManager.h)
qt5_wrap_cpp(SCENE_MANAGER_SOURCE_FILES ${SCENE_MANAGER_SOURCE_FILES})
qt5_wrap_ui(SCENE_MANAGER_HEADERS_MOC ../../ui/SceneManager.ui)
set(TARGETS ${TARGETS} OperatorSceneManager)
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


pkg_check_modules(GSTREAMER-APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GSTREAMER-RTP REQUIRED gstreamer-rtp-1.0)
set(TARGETS ${TARGETS} OperatorRtspClients)
add_executable(OperatorRtspClients
    RtspClientsNode.cpp
    RtspClients.h
    RtspClients.cpp)
add_dependencies(OperatorRtspClients ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OperatorRtspClients
    ${catkin_LIBRARIES} ${GSTREAMER_LIBRARIES} ${GSTREAMER-RTP_LIBRARIES} ${GSTREAMER-APP_LIBRARIES} pthread)
target_include_directories(OperatorRtspClients
    PRIVATE ${catkin_INCLUDE_DIRS} ${GSTREAMER_INCLUDE_DIRS} ${GSTREAMER-RTP_INCLUDE_DIRS} ${GSTREAMER-APP_INCLUDE_DIRS})


set(TARGETS ${TARGETS} OperatorVideoConfigSend)
add_executable(OperatorVideoConfigSend VideoConfigSendNode.cpp)
add_dependencies(OperatorVideoConfigSend ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OperatorVideoConfigSend ${catkin_LIBRARIES} PahoMqttCpp::paho-mqttpp3)
target_include_directories(OperatorVideoConfigSend
    PRIVATE ${catkin_INCLUDE_DIRS})


set(TARGETS ${TARGETS} OperatorBitrateConfigSend)
add_executable(OperatorBitrateConfigSend BitrateConfigSendNode.cpp)
add_dependencies(OperatorBitrateConfigSend ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OperatorBitrateConfigSend ${catkin_LIBRARIES} PahoMqttCpp::paho-mqttpp3)
target_include_directories(OperatorBitrateConfigSend
    PRIVATE ${catkin_INCLUDE_DIRS})


set(TARGETS ${TARGETS} OperatorBitrateIntegrator)
add_executable(OperatorBitrateIntegrator BitrateIntegratorNode.cpp)
add_dependencies(OperatorBitrateIntegrator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OperatorBitrateIntegrator ${catkin_LIBRARIES})
target_include_directories(OperatorBitrateIntegrator
    PRIVATE ${catkin_INCLUDE_DIRS})


set(TARGETS ${TARGETS} OperatorBitratePredictionReceiver)
add_executable(OperatorBitratePredictionReceiver BitratePredictionReceiver.cpp)
target_link_libraries(OperatorBitratePredictionReceiver ${catkin_LIBRARIES})
target_include_directories(OperatorBitratePredictionReceiver
    PRIVATE ${catkin_INCLUDE_DIRS})


install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
