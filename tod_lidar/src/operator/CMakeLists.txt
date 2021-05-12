cmake_minimum_required(VERSION 2.8.3)

set(TARGETS ${TARGETS} OperatorLaserScansReceiver)
add_executable(OperatorLaserScansReceiver OperatorLaserScansReceiver.cpp)
target_link_libraries(OperatorLaserScansReceiver ${catkin_LIBRARIES})
target_include_directories(OperatorLaserScansReceiver PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET OperatorLaserScansReceiver PROPERTY CXX_STANDARD 17)

set(TARGETS ${TARGETS} OperatorObjectListsReceiver)
add_executable(OperatorObjectListsReceiver OperatorObjectListsReceiver.cpp)
target_link_libraries(OperatorObjectListsReceiver ${catkin_LIBRARIES})
target_include_directories(OperatorObjectListsReceiver PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET OperatorObjectListsReceiver PROPERTY CXX_STANDARD 17)

set(TARGETS ${TARGETS} OperatorObjectMarkersReceiver)
add_executable(OperatorObjectMarkersReceiver OperatorObjectMarkersReceiver.cpp)
target_link_libraries(OperatorObjectMarkersReceiver ${catkin_LIBRARIES})
target_include_directories(OperatorObjectMarkersReceiver PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET OperatorObjectMarkersReceiver PROPERTY CXX_STANDARD 17)

install(TARGETS ${TARGETS}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
