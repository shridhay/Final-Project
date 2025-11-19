cmake_minimum_required(VERSION 3.5)
project(shutter_face_ros)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ROS 2 interface generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PupilsLocation.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Install Python package 'shutter_face' (vendored here)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import sys; v=sys.version_info; print(f'{v.major}.{v.minor}')"
  OUTPUT_VARIABLE PYTHON_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
# Install inner shutter_face package contents to site-packages
install(DIRECTORY shutter_face/shutter_face/
  DESTINATION lib/python${PYTHON_VERSION}/site-packages/shutter_face
  FILES_MATCHING 
  PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# Install executables
install(PROGRAMS
  src/gaze_master.py
  src/simple_face.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install data
install(DIRECTORY shutter_face/shutter_face/data
  DESTINATION share/${PROJECT_NAME}/shutter_face/shutter_face
)

# Install launch files (ROS 1 XML kept for reference; ROS 2 launch added separately)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
