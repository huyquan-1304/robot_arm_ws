cmake_minimum_required(VERSION 3.8)

project(robot_arm_4_dof)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    urdf
    meshes
    textures
    config
    worlds
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS node/pick_and_place_run.py
     DESTINATION lib/${PROJECT_NAME})
ament_package()
