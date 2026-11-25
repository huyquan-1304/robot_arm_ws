cmake_minimum_required(VERSION 3.8)
project(amm_base_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    meshes
    rviz
    urdf
    worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/amm_teleop_key.py
    scripts/imu_bias_calibrator.py
    scripts/odom_error_monitor.py
    scripts/base_bridge_node.py
    scripts/odom_check.py
    scripts/wit_imu_node.py
    scripts/wit_imu_config.py
    scripts/heading_hold_node.py
    scripts/goto_test.py
    scripts/kinematics_report.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
