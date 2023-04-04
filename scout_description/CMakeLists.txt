cmake_minimum_required(VERSION 3.5)
project(scout_description)
find_package(ament_cmake REQUIRED)
find_package(roslaunch REQUIRED)

ament_package()

# roslaunch_add_file_check(launch)

install(
  DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)
