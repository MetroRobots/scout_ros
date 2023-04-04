cmake_minimum_required(VERSION 3.5)
project(scout_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roslaunch REQUIRED)
find_package(scout_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ugv_sdk REQUIRED)

# find_package(Boost REQUIRED COMPONENTS chrono)

###################################
## catkin specific configuration ##
###################################

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
)

add_library(scout_messenger STATIC src/scout_messenger.cpp)
ament_target_dependencies(scout_messenger "tf2_ros" "nav_msgs" "tf2_ros" "rclcpp" "sensor_msgs" "scout_msgs")

add_executable(scout_base_node src/scout_base_node.cpp)
ament_target_dependencies(scout_base_node scout_messenger "tf2_ros" "nav_msgs" "tf2_ros" "rclcpp" "sensor_msgs" "scout_msgs")
ament_export_include_directories(include)
ament_export_libraries(scout_messenger)
ament_export_dependencies(ugv_sdk scout_msgs)
ament_package()

#############
## Install ##
#############

install(TARGETS scout_messenger scout_base_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
