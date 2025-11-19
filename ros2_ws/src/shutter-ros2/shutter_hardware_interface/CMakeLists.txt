cmake_minimum_required(VERSION 3.16)
project(shutter_hardware_interface)

# ---- Build settings ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---- Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(dynamixel_workbench_msgs REQUIRED)
find_package(joint_limits REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---- Library with your core HW code ----
add_library(shutter_hw_core SHARED
  src/dynamixel_ros_control.cpp
  src/shutter_position_interface.cpp
)

target_include_directories(shutter_hw_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(shutter_hw_core
  rclcpp
  sensor_msgs
  std_msgs
  dynamixel_workbench_toolbox
  dynamixel_workbench_msgs
  joint_limits
)

target_link_libraries(shutter_hw_core
  yaml-cpp
)

# ---- Node executable ----
add_executable(shutter_position_node
  src/shutter_position_node.cpp
)

target_link_libraries(shutter_position_node
  shutter_hw_core
)

ament_target_dependencies(shutter_position_node
  rclcpp
)

# ---- Install ----
install(TARGETS
  shutter_hw_core
  shutter_position_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ---- Exports ----
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  dynamixel_workbench_toolbox
  dynamixel_workbench_msgs
  joint_limits
  yaml-cpp
)

ament_package()
