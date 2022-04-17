cmake_minimum_required(VERSION 3.0.2)

set(TARGETS ${TARGETS} LaserScansSender)
add_executable(LaserScansSender LaserScansSender.cpp)
target_link_libraries(LaserScansSender ${catkin_LIBRARIES})
target_include_directories(LaserScansSender PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET LaserScansSender PROPERTY CXX_STANDARD 17)

set(TARGETS ${TARGETS} ObjectListsSender)
add_executable(ObjectListsSender ObjectListsSender.cpp)
target_link_libraries(ObjectListsSender ${catkin_LIBRARIES})
target_include_directories(ObjectListsSender PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET ObjectListsSender PROPERTY CXX_STANDARD 17)

set(TARGETS ${TARGETS} LaserScansDetector)
add_executable(LaserScansDetector LaserScansDetector.cpp LaserScansDetectorNode.cpp)
target_link_libraries(LaserScansDetector ${catkin_LIBRARIES})
target_include_directories(LaserScansDetector PUBLIC ${catkin_INCLUDE_DIRS})
set_property(TARGET LaserScansDetector PROPERTY CXX_STANDARD 17)

set(TARGETS ${TARGETS} GridMapCreator)
add_executable(GridMapCreator GridMapCreator.cpp GridMapCreatorNode.cpp)
target_link_libraries(GridMapCreator ${catkin_LIBRARIES})
target_include_directories(GridMapCreator PUBLIC ${catkin_INCLUDE_DIRS})
set_property(TARGET GridMapCreator PROPERTY CXX_STANDARD 17)

set(TARGETS ${TARGETS} GridMapDetector)
add_executable(GridMapDetector GridMapDetector.cpp GridMapDetectorNode.cpp
                ../../external/SimpleDBSCAN/kdtree.cpp)
target_link_libraries(GridMapDetector ${catkin_LIBRARIES})
target_include_directories(GridMapDetector PUBLIC ${catkin_INCLUDE_DIRS} ../../external)
set_property(TARGET GridMapDetector PROPERTY CXX_STANDARD 17)

install(TARGETS ${TARGETS}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
