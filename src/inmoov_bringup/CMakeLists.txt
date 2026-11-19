cmake_minimum_required(VERSION 3.8)
project(inmoov_bringup)

find_package(ament_cmake REQUIRED)

# Install configuration and launch files
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Install executable Python scripts (e.g., idle_animator)
install(PROGRAMS
  tools/idle_animator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
