cmake_minimum_required(VERSION 3.0.2)
project(traffic_light)


# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  darknet_ros_msgs
  message_generation
  roscpp
  std_msgs
  #waypoint_maker
)




add_message_files(
   FILES
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(traffic_light src/traffic_light.cpp)
target_link_libraries(traffic_light ${catkin_LIBRARIES})
