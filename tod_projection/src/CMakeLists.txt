find_package(OpenCV REQUIRED)

set(TARGETS ${TARGETS} OperatorVideoProjection)
add_executable(OperatorVideoProjection
    OperatorVideoProjectionNode.cpp
    OperatorVideoProjection.cpp)
add_dependencies(OperatorVideoProjection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OperatorVideoProjection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} pthread)
target_include_directories(OperatorVideoProjection PRIVATE ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

set(TARGETS ${TARGETS} OperatorLaneProjection)
add_executable(OperatorLaneProjection OperatorLaneProjectionNode.cpp)
add_dependencies(OperatorLaneProjection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OperatorLaneProjection ${catkin_LIBRARIES})
target_include_directories(OperatorLaneProjection PRIVATE ${catkin_INCLUDE_DIRS})

install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
