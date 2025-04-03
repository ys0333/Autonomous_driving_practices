cmake_minimum_required(VERSION 3.0.2)
project(new_lane)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
  ackermann_msgs
  geometry_msgs
  message_generation
  message_runtime
  laser_geometry
)

add_message_files(
   FILES
   lidar_topic_msg.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   ackermann_msgs
   sensor_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}./include
)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES new_lane
    CATKIN_DEPENDS roscpp rospy std_msgs
)

add_executable(lane_changer_node 
   src/lane_changer.cpp 
   src/cubic_spline_planner.cpp
   src/frenet_optimal_trajectory.cpp
   src/polynomials.cpp
)
add_dependencies(lane_changer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(lane_changer_node
   ${catkin_LIBRARIES}
)
