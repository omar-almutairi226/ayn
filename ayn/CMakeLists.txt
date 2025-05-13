cmake_minimum_required(VERSION 3.8)
project(ayn)

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

install(DIRECTORY urdf launch worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
