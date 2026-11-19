cmake_minimum_required(VERSION 3.8)
project(inmoov_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY tools/ DESTINATION lib/${PROJECT_NAME})
  tools/idle_animator.py
  DESTINATION lib/${PROJECT_NAME}
)
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
