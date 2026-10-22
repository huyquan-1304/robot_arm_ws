#!/usr/bin/env python3
"""Mo phong robot_arm_4_dof trong Gazebo Harmonic + dieu khien bang MoveIt 2 (ROS 2 Jazzy).

Chay:
  ros2 launch robot_arm_4_dof simulation_robot.launch.py

KHONG chay kem demo.launch.py. File nay da tu bat move_group va RViz.
Neu chay ca hai se co 2 node /controller_manager xung dot nhau.

Thu tu khoi dong:
  1. Gazebo (empty.sdf, -r de mo phong tu chay, OGRE 1 cho VMware)
  2. Cau noi /clock tu Gazebo sang ROS
  3. robot_state_publisher voi URDF use_gazebo:=true
  4. Dua robot vao Gazebo (sau 3 giay)
  5. Sau khi robot vao Gazebo: bat joint_state_broadcaster
  6. Sau do: bat arm_controller va gripper_controller
  7. Sau khi arm_controller chay: bat move_group va RViz
"""

import os
import tempfile
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


PACKAGE_NAME = 'robot_arm_4_dof'
MOVEIT_PACKAGE = 'moveit_robot_arm'
ROBOT_NAME = 'robot_arm_4_dof'
# [PICK PLACE] Ten world phai trung voi <world name=...> trong worlds/pick_place.sdf
WORLD_NAME = 'pick_place'


def generate_launch_description():
    package_share = Path(get_package_share_directory(PACKAGE_NAME))
    moveit_share = Path(get_package_share_directory(MOVEIT_PACKAGE))
    gazebo_share = Path(get_package_share_directory('ros_gz_sim'))

    xacro_file = package_share / 'urdf' / 'robot_arm_4_dof.urdf.xacro'
    meshes_dir = package_share / 'meshes'
    controllers_file = package_share / 'config' / 'final.yaml'
    moveit_controllers_file = package_share / 'config' / 'moveit_controllers_gazebo.yaml'
    rviz_file = moveit_share / 'config' / 'moveit.rviz'
    world_file = package_share / 'worlds' / f'{WORLD_NAME}.sdf'

    for path, hint in [
        (xacro_file, 'install DIRECTORY urdf'),
        (meshes_dir, 'install DIRECTORY meshes'),
        (controllers_file, 'install DIRECTORY config, va co file final.yaml'),
        (moveit_controllers_file, 'install DIRECTORY config, va co file moveit_controllers_gazebo.yaml'),
        (world_file, 'install DIRECTORY worlds, va co file pick_place.sdf'),
    ]:
        if not path.exists():
            raise FileNotFoundError(
                f'Khong tim thay {path}. Kiem tra CMakeLists.txt cua {PACKAGE_NAME}: {hint}.'
            )

    # ------------------------------------------------------------------
    # URDF
    # [SUA] use_gazebo:=true de bat khoi ros2_control gz_ros2_control va plugin Gazebo.
    # Ban cu goi xacro khong co tham so nen URDF khong co phan dieu khien.
    # ------------------------------------------------------------------
    robot_description = xacro.process_file(
        str(xacro_file), mappings={'use_gazebo': 'true'}
    ).toxml()

    # Ban rieng cho Gazebo: doi package:// thanh file:/// tuyet doi (giu cach cu cua ban,
    # vi Gazebo Harmonic trong VMware khong phan giai duoc URI package://).
    # robot_state_publisher, move_group, RViz van dung ban package:// goc.
    mesh_uri = meshes_dir.as_uri().rstrip('/') + '/'
    robot_description_gz = robot_description.replace(
        f'package://{PACKAGE_NAME}/meshes/', mesh_uri
    ).replace(
        f'model://{PACKAGE_NAME}/meshes/', mesh_uri
    )
    urdf_path = Path(tempfile.gettempdir()) / f'{ROBOT_NAME}_harmonic.urdf'
    urdf_path.write_text(robot_description_gz, encoding='utf-8')

    # ------------------------------------------------------------------
    # Cau hinh MoveIt
    # [MOI] Lay URDF ban Gazebo (khong co FakeSystem) va file controller rieng cho Gazebo.
    # Khong dong cham file nao cua moveit_robot_arm, nen demo.launch.py van chay nhu cu.
    # ------------------------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder(ROBOT_NAME, package_name=MOVEIT_PACKAGE)
        .robot_description(file_path=str(xacro_file), mappings={'use_gazebo': 'true'})
        .trajectory_execution(file_path=str(moveit_controllers_file))
        .to_moveit_configs()
    )

    # ------------------------------------------------------------------
    # Gazebo
    # [SUA] Them -r de mo phong tu chay. Ban cu mo Gazebo o trang thai tam dung,
    # dong ho /clock dung yen thi controller khong chay duoc.
    # ------------------------------------------------------------------
    resource_path = os.pathsep.join(filter(None, [
        str(package_share.parent),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / 'launch' / 'gz_sim.launch.py')),
        launch_arguments={
            # [PICK PLACE] Dung file world tuy chinh thay cho empty.sdf
            'gz_args': f'-r -v 3 {world_file} --render-engine ogre',
        }.items(),
    )

    # [MOI] Noi dong ho mo phong Gazebo sang ROS. Thieu buoc nay thi cac node
    # dung use_sim_time se dung yen o thoi diem 0.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot_arm_4_dof',
        arguments=[
            '-world', WORLD_NAME,
            '-file', str(urdf_path),
            '-name', ROBOT_NAME,
            # [SUA] -z 0 de robot trong Gazebo trung do cao voi MoveIt.
            # Ban cu -z 0.05 lam robot cao hon 5 cm, kep se ha hut vat.
            '-x', '0', '-y', '0', '-z', '0',
        ],
        output='screen',
    )

    # ------------------------------------------------------------------
    # [MOI] Bat controller cua gz_ros2_control (doc tu final.yaml)
    # ------------------------------------------------------------------
    def spawner(controller_name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                controller_name,
                '--controller-manager', '/controller_manager',
                '--controller-manager-timeout', '60',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )

    jsb_spawner = spawner('joint_state_broadcaster')
    arm_spawner = spawner('arm_controller')
    gripper_spawner = spawner('gripper_controller')

    # ------------------------------------------------------------------
    # [MOI] MoveIt + RViz, dung thoi gian mo phong
    # ------------------------------------------------------------------
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': True,
                'publish_robot_description_semantic': True,
            },
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_file)],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gazebo,
        clock_bridge,
        state_publisher,
        TimerAction(period=3.0, actions=[spawn_robot]),

        # Robot vao Gazebo xong moi bat joint_state_broadcaster
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=[jsb_spawner],
        )),
        # joint_state_broadcaster chay xong moi bat controller tay va kep
        RegisterEventHandler(OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[arm_spawner, gripper_spawner],
        )),
        # Controller tay da san sang moi bat MoveIt va RViz
        RegisterEventHandler(OnProcessExit(
            target_action=arm_spawner,
            on_exit=[move_group, rviz],
        )),
    ])
