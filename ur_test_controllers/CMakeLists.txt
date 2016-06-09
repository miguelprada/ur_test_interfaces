cmake_minimum_required(VERSION 2.8.3)
project(ur_test_controllers)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  realtime_tools
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library( ${PROJECT_NAME}
  src/ur_test_nullvel_controller.cpp
)

target_link_libraries( ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
