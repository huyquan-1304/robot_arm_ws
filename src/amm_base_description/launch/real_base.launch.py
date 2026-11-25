#!/usr/bin/env python3
"""Chay XE THAT (4 banh, chua IMU) tren ROS 2 Jazzy. STM32 thay cho Gazebo, phan con lai giong mo phong.

  /cmd_vel → mecanum_drive_controller → /base/wheel_commands → base_bridge → STM32 → 4 motor
  4 encoder → STM32 → base_bridge → /base/wheel_states → mecanum_drive_controller → /odom + TF

IMU + EKF (tham so use_ekf, chi voi hw:=real):
  use_ekf:=false  (mac dinh) controller phat /odom va TF odom → base_footprint, giong luc chua co IMU.
  use_ekf:=true   them wit_imu_node (IMU USB) → imu_bias_calibrator → ekf_filter_node.
                  EKF phat /odometry/filtered va TF. Controller tat TF.
                  DE XE DUNG YEN ~5 s sau khi khoi dong: calibrator do bias con quay.
  imu_port:=/dev/ttyUSB0

GIU HUONG (tham so heading_hold, can use_ekf:=true):
  heading_hold:=true  them heading_hold_node: /cmd_vel_raw → (bu xoay bang IMU) → /cmd_vel.
                      Teleop / Nav2 phai phat len /cmd_vel_raw (teleop: -p topic:=/cmd_vel_raw).

Che do (tham so hw):
  hw:=real      Xe that qua USB (port:=/dev/ttyACM0). Mac dinh TAT, phai goi ~/enable.
  hw:=loopback  Gia lap STM32 trong node base_bridge, khong can phan cung. Tu BAT.
                Dung de thu toan bo phan ROS tren may ao truoc khi ra xe.

Chay:
  ros2 launch amm_base_description real_base.launch.py                       # xe that
  ros2 launch amm_base_description real_base.launch.py hw:=loopback rviz:=true
  ros2 launch amm_base_description real_base.launch.py use_ekf:=true       # xe that + IMU + EKF
  ros2 launch amm_base_description real_base.launch.py use_ekf:=true heading_hold:=true

  # Terminal 2, xe that: bat xe
  ros2 service call /base_bridge/enable std_srvs/srv/SetBool "{data: true}"
  # Terminal 3: lai xe (dong ho that, toc do vua phai)
  ros2 run amm_base_description amm_teleop_key.py --ros-args -p use_sim:=false -p linear:=0.08 -p angular:=0.35
  # Terminal 4: xem odometry so voi diem xuat phat
  ros2 run amm_base_description odom_check.py

Thu tu khoi dong:
  1. robot_state_publisher, base_bridge, ros2_control_node
  2. Sau 3 s (cho STM32 gui so xung): joint_state_broadcaster
  3. Sau do: mecanum_drive_controller
  4. RViz (neu rviz:=true), fixed frame = odom
"""

from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = 'amm_base_description'


def launch_setup(context, *args, **kwargs):
    hw = LaunchConfiguration('hw').perform(context).lower()
    if hw not in ('real', 'loopback'):
        raise RuntimeError(f'hw:={hw} khong hop le. Chon real hoac loopback')
    port = LaunchConfiguration('port').perform(context)
    use_rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    auto_enable = LaunchConfiguration('auto_enable').perform(context).lower()
    auto_enable = (hw == 'loopback') if auto_enable == 'auto' else auto_enable == 'true'
    use_ekf = LaunchConfiguration('use_ekf').perform(context).lower() == 'true'
    imu_port = LaunchConfiguration('imu_port').perform(context)
    if use_ekf and hw != 'real':
        raise RuntimeError('use_ekf:=true can IMU that, chi dung voi hw:=real')
    heading_hold = LaunchConfiguration('heading_hold').perform(context).lower() == 'true'
    if heading_hold and not use_ekf:
        raise RuntimeError('heading_hold:=true can use_ekf:=true (lay huong tu EKF + IMU)')

    share = Path(get_package_share_directory(PACKAGE_NAME))
    config = share / 'config'

    robot_description = xacro.process_file(
        str(share / 'urdf' / 'amm_base.urdf.xacro'),
        mappings={'drive_mode': 'real', 'use_ekf': str(use_ekf).lower()},
    ).toxml()

    actions = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
            output='screen',
        ),
        Node(
            package=PACKAGE_NAME,
            executable='base_bridge_node.py',
            name='base_bridge',
            parameters=[str(config / 'base_bridge.yaml'), {
                'port': port,
                'loopback': hw == 'loopback',
                'auto_enable': auto_enable,
            }],
            output='screen',
        ),
        # Jazzy: controller_manager tu nhan URDF qua topic /robot_description
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                str(config / 'amm_base_controllers.yaml'),
                str(config / 'amm_base_controllers_real.yaml'),   # ghi de: gioi han toc do xe that
                *([str(config / 'amm_base_controllers_ekf.yaml')] if use_ekf else []),  # EKF phat TF
            ],
            # Cung ten topic voi mo phong: teleop, RViz, Nav2 khong phai sua
            remappings=[
                ('/mecanum_drive_controller/reference', '/cmd_vel'),
                ('/mecanum_drive_controller/odometry', '/odom'),
                ('/mecanum_drive_controller/tf_odometry', '/tf'),
            ],
            output='screen',
        ),
    ]

    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '30'],
            output='screen',
        )

    jsb = spawner('joint_state_broadcaster')
    mecanum = spawner('mecanum_drive_controller')
    actions += [
        # Cho STM32 gui so xung truoc khi bat controller
        TimerAction(period=3.0, actions=[jsb]),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[mecanum])),
    ]

    if use_ekf:
        actions.append(Node(
            package=PACKAGE_NAME,
            executable='wit_imu_node.py',
            name='wit_imu',
            # gyro_scale: bai xoay 5 vong (25/11/2026) cho thay con quay khop goc that, khong can hieu chinh
            parameters=[{'port': imu_port, 'baud': 115200, 'frame_id': 'imu_link', 'gyro_scale': 1.0}],
            output='screen',
        ))
        calibrator = Node(
            package=PACKAGE_NAME,
            executable='imu_bias_calibrator.py',
            parameters=[{
                'input_topic': '/imu/data_raw',
                'output_topic': '/imu/corrected',
                'frame_id': 'imu_link',
                'use_sim': False,
            }],
            output='screen',
        )
        ekf = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[str(config / 'ekf.yaml'), {'use_sim_time': False}],
            output='screen',
        )
        # Sau khi controller chay (/odom co), calibrator cho xe dung yen roi do bias
        after_mecanum = [calibrator, ekf]
        if heading_hold:
            after_mecanum.append(Node(
                package=PACKAGE_NAME,
                executable='heading_hold_node.py',
                name='heading_hold',
                parameters=[{'hold_heading': True, 'rate_loop': True}],
                output='screen',
            ))
        actions.append(RegisterEventHandler(OnProcessExit(target_action=mecanum, on_exit=after_mecanum)))

    if use_rviz:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(share / 'rviz' / 'amm_base.rviz')],
            parameters=[{'use_sim_time': False}],
            output='screen',
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hw', default_value='real',
                              description='real: xe that qua USB | loopback: gia lap STM32 trong node'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='Cong USB CDC cua STM32 (chi dung khi hw:=real)'),
        DeclareLaunchArgument('auto_enable', default_value='auto',
                              description='auto: loopback tu BAT, xe that phai goi ~/enable | true | false'),
        DeclareLaunchArgument('use_ekf', default_value='false',
                              description='true: them IMU (USB) + hieu chuan bias + EKF (chi hw:=real)'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB0', description='Cong USB cua IMU'),
        DeclareLaunchArgument('heading_hold', default_value='false',
                              description='true: giu huong bang IMU (can use_ekf:=true). Teleop phat len /cmd_vel_raw'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Mo RViz tren may nay (Pi khong man hinh thi de false)'),
        OpaqueFunction(function=launch_setup),
    ])
