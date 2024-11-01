cmake_minimum_required(VERSION 2.8.3)
project(smabo)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
  FILES
  Floats.msg
)

catkin_python_setup()

