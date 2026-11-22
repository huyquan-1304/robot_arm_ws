#!/usr/bin/env python3
"""Mo phong xe AMM trong Gazebo Harmonic (ROS 2 Jazzy).

Chay:
  ros2 launch amm_base_description gazebo.launch.py                     # 4 banh quay that (ros2_control)
  ros2 launch amm_base_description gazebo.launch.py drive_mode:=velocity  # che do ngay 1
  ros2 launch amm_base_description gazebo.launch.py rviz:=false
  ros2 launch amm_base_description gazebo.launch.py use_ekf:=false      # khong EKF, controller phat TF

Che do control + use_ekf:=true (mac dinh) chay them:
  imu_bias_calibrator : /imu → tru bias, dien covariance → /imu/corrected
  ekf_filter_node     : /odom + /imu/corrected → /odometry/filtered + TF odom → base_footprint

Lai xe (terminal khac):
  drive_mode:=control  : ros2 run amm_base_description amm_teleop_key.py
  drive_mode:=velocity : ros2 run amm_base_description amm_teleop_key.py --ros-args -p stamped:=false

So sai so odometry (terminal khac, chi che do control):
  ros2 run amm_base_description odom_error_monitor.py

Thu tu khoi dong (che do control):
  1. Gazebo voi world amm_world.sdf (-r de tu chay, OGRE 1 cho VMware)
  2. robot_state_publisher + ros_gz_bridge
  3. Dua xe vao Gazebo (sau 3 giay). Plugin gz_ros2_control tao /controller_manager
  4. Xe vao xong: bat joint_state_broadcaster
  5. Sau do: bat mecanum_drive_controller
  6. RViz, fixed frame = odom
"""

import os
import tempfile
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = 'amm_base_description'
ROBOT_NAME = 'amm_base'          # phai trung voi gz_bridge_velocity.yaml
WORLD_NAME = 'amm_world'         # phai trung voi <world name> trong file .sdf
DRIVE_MODES = ('control', 'velocity')


def launch_setup(context, *args, **kwargs):
    drive_mode = LaunchConfiguration('drive_mode').perform(context)
    if drive_mode not in DRIVE_MODES:
        raise ValueError(f'drive_mode phai la mot trong {DRIVE_MODES}, nhan duoc "{drive_mode}"')
    use_ekf_arg = LaunchConfiguration('use_ekf').perform(context).lower()
    if use_ekf_arg not in ('true', 'false'):
        raise ValueError(f'use_ekf phai la true hoac false, nhan duoc "{use_ekf_arg}"')
    use_ekf = drive_mode == 'control' and use_ekf_arg == 'true'

    share = Path(get_package_share_directory(PACKAGE_NAME))
    gazebo_share = Path(get_package_share_directory('ros_gz_sim'))

    xacro_file = share / 'urdf' / 'amm_base.urdf.xacro'
    world_file = share / 'worlds' / f'{WORLD_NAME}.sdf'
    meshes_dir = share / 'meshes'
    rviz_file = share / 'rviz' / 'amm_base.rviz'
    bridge_files = [
        share / 'config' / 'gz_bridge_sensors.yaml',
        share / 'config' / f'gz_bridge_{drive_mode}.yaml',
    ]
    controllers_file = share / 'config' / 'amm_base_controllers.yaml'
    ekf_file = share / 'config' / 'ekf.yaml'

    for path, hint in [
        *([(ekf_file, 'install DIRECTORY config'),
           (share / 'config' / 'amm_base_controllers_ekf.yaml', 'install DIRECTORY config')]
          if use_ekf else []),
        (xacro_file, 'install DIRECTORY urdf'),
        (meshes_dir, 'install DIRECTORY meshes'),
        (world_file, 'install DIRECTORY worlds'),
        (rviz_file, 'install DIRECTORY rviz'),
        (controllers_file, 'install DIRECTORY config'),
        *[(f, 'install DIRECTORY config') for f in bridge_files],
    ]:
        if not path.exists():
            raise FileNotFoundError(
                f'Khong tim thay {path}. Kiem tra CMakeLists.txt: {hint}, roi colcon build lai.'
            )

    robot_description = xacro.process_file(
        str(xacro_file), mappings={'drive_mode': drive_mode, 'use_ekf': str(use_ekf).lower()}).toxml()

    # Ban rieng cho Gazebo: doi package:// thanh file:/// tuyet doi
    # (giong project canh tay, Gazebo Harmonic tren VMware khong phan giai duoc package://).
    # robot_state_publisher va RViz van dung ban package:// goc.
    mesh_uri = meshes_dir.as_uri().rstrip('/') + '/'
    robot_description_gz = robot_description.replace(
        f'package://{PACKAGE_NAME}/meshes/', mesh_uri)
    urdf_gz_path = Path(tempfile.gettempdir()) / f'{ROBOT_NAME}_harmonic.urdf'
    urdf_gz_path.write_text(robot_description_gz, encoding='utf-8')

    resource_path = os.pathsep.join(filter(None, [
        str(share.parent),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / 'launch' / 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -v 3 {world_file} --render-engine ogre',
        }.items(),
    )

    # gz_ros2_control doc robot_description tu node nay (ten mac dinh robot_state_publisher)
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', WORLD_NAME,
            '-file', str(urdf_gz_path),
            '-name', ROBOT_NAME,
            '-x', '0', '-y', '0', '-z', '0.01',
        ],
        output='screen',
    )

    bridges = [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_{f.stem}',
            parameters=[{'config_file': str(f), 'use_sim_time': True}],
            output='screen',
        )
        for f in bridge_files
    ]

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(rviz_file), '-f', 'odom'],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gazebo,
        state_publisher,
        *bridges,
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=5.0, actions=[rviz]),
    ]

    if drive_mode == 'control':
        def spawner(name):
            return Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    name,
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '60',
                ],
                output='screen',
            )

        jsb_spawner = spawner('joint_state_broadcaster')
        mecanum_spawner = spawner('mecanum_drive_controller')

        actions += [
            # Xe vao Gazebo xong (co /controller_manager) moi bat joint_state_broadcaster
            RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[jsb_spawner])),
            # Sau do bat controller Mecanum
            RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[mecanum_spawner])),
        ]

    if use_ekf:
        imu_calibrator = Node(
            package=PACKAGE_NAME,
            executable='imu_bias_calibrator.py',
            parameters=[{
                'input_topic': '/imu',
                'output_topic': '/imu/corrected',
                'use_sim': True,
            }],
            output='screen',
        )
        ekf = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[str(ekf_file)],
            output='screen',
        )
        # Bat cung luc voi controller: calibrator cho /odom bao dung yen roi moi do bias
        actions += [
            RegisterEventHandler(OnProcessExit(
                target_action=jsb_spawner, on_exit=[imu_calibrator, ekf])),
        ]

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Mo RViz cung Gazebo'),
        DeclareLaunchArgument(
            'drive_mode', default_value='control',
            description='control: 4 banh quay qua ros2_control | velocity: keo thang than xe'),
        DeclareLaunchArgument(
            'use_ekf', default_value='true',
            description='true: hieu chuan IMU + EKF phat TF | false: controller phat TF (chi che do control)'),
        OpaqueFunction(function=launch_setup),
    ])
