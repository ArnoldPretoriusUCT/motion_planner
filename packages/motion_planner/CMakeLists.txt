cmake_minimum_required(VERSION 2.8.3)
project(motion_planner)
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    duckietown_msgs
    image_transport
    message_filters
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
)
catkin_package()
find_package(apriltag REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(apriltag_detector_node src/apriltag_detector_node.cpp)
add_dependencies(apriltag_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${OpenCV_EXPORTED_TARGETS})
target_link_libraries(apriltag_detector_node ${catkin_LIBRARIES} apriltag ${OpenCV_LIBRARIES})

add_executable(control_node src/control_node.cpp)
add_dependencies(control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_node ${catkin_LIBRARIES})

add_executable(extended_kalman_filter_node src/extended_kalman_filter_node.cpp)
add_dependencies(extended_kalman_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(extended_kalman_filter_node ${catkin_LIBRARIES})

add_executable(odometry_node src/odometry_node.cpp)
add_dependencies(odometry_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometry_node ${catkin_LIBRARIES})

add_executable(planning_node src/planning_node.cpp)
add_dependencies(planning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_node ${catkin_LIBRARIES})
