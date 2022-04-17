cmake_minimum_required(VERSION 3.0.2)

set(TARGETS ${TARGETS} LaserScansReceiver)
add_executable(LaserScansReceiver LaserScansReceiver.cpp)
target_link_libraries(LaserScansReceiver ${catkin_LIBRARIES})
target_include_directories(LaserScansReceiver PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET LaserScansReceiver PROPERTY CXX_STANDARD 17)

set(TARGETS ${TARGETS} ObjectListsReceiver)
add_executable(ObjectListsReceiver ObjectListsReceiver.cpp)
target_link_libraries(ObjectListsReceiver ${catkin_LIBRARIES})
target_include_directories(ObjectListsReceiver PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET ObjectListsReceiver PROPERTY CXX_STANDARD 17)

install(TARGETS ${TARGETS}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
