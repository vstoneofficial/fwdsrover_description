cmake_minimum_required(VERSION 3.8)
project(fwdsrover_description)

find_package(ament_cmake REQUIRED)

message(STATUS ">>> Installing rover_twist_relay.py into lib/${PROJECT_NAME}")

# --------------------------------------------------
# Python node
# --------------------------------------------------
install(
  PROGRAMS
    ${CMAKE_CURRENT_SOURCE_DIR}/params/rover_twist_relay.py
    ${CMAKE_CURRENT_SOURCE_DIR}/params/joint_state_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------
# Resource directories
# --------------------------------------------------
install(DIRECTORY
  launch
  meshes
  rviz
  urdf
  params
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
