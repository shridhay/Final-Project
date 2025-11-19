cmake_minimum_required(VERSION 3.8)
project(shutter_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  src/unity_simulation_manager.py
  config/infer_webcam_id.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    src/motor_startup_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

######################
## Detect Webcam ID ##
######################
# if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config/infer_webcam_id.py")
#  execute_process(COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/config/infer_webcam_id.py
#                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config)
#endif()

#if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config/env-hooks/webcam_id.bash")
#  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/config/env-hooks/webcam_hook.bash")
#endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
