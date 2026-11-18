cmake_minimum_required(VERSION 3.8)
project(inmoov_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch robots urdf config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES urdf.rviz scripts/jaw_state_merger.py
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/jaw_to_joint_states.py
  scripts/face_tracker_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
