cmake_minimum_required(VERSION 2.8.3)
project(smabo)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
)

add_service_files(
  FILES
  TextFilter.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs sensor_msgs tf2_ros
)

