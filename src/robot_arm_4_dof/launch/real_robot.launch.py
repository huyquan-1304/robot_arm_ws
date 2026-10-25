#!/usr/bin/env python3
"""Chay robot_arm_4_dof voi robot that, hoac voi STM32 gia lap de xem truoc, tren ROS 2 Jazzy.

Vi tri: robot_arm_4_dof/launch/real_robot.launch.py

3 che do (tham so hw):
  hw:=emulator  STM32 GIA LAP. Node stm32_emulator chay lai logic firmware (ros_link.c, joint.c:
                gioi han khop, toc do, lam tron 0.1 RPM, watchdog) va dong co ZDT, noi voi node cau
                noi qua cong noi tiep ao /tmp/ttySTM32. Code tren Pi chay y het robot that,
                chi khac cong noi tiep. Dung de XEM TRUOC khi nap STM32.
  hw:=real      Robot that qua USB-UART (port:=/dev/ttyUSB0). Phai goi ~/enable bang tay.
  hw:=loopback  Gia lap tho trong node cau noi (chi kiem tra duong ROS, khong mo phong firmware).

Xem truoc tren may ao (co man hinh):
  ros2 launch robot_arm_4_dof real_robot.launch.py hw:=emulator use_rviz:=true
  # Terminal 2:
  ros2 run robot_arm_4_dof pick_and_place_run.py --ros-args -p real_limits:=true
  # Thu voi gioi han firmware gia dinh da noi rong:
  ros2 launch robot_arm_4_dof real_robot.launch.py hw:=emulator use_rviz:=true relax_limits:=true

Robot that:
  ros2 launch robot_arm_4_dof real_robot.launch.py hw:=real port:=/dev/ttyUSB0
  ros2 service call /stm32_bridge/enable std_srvs/srv/SetBool "{data: true}"
  ros2 run robot_arm_4_dof pick_and_place_run.py --ros-args -p real_limits:=true

Trong RViz (twin:=true, mac dinh):
  robot dac   = vi tri THUC STM32 tra ve
  bong ma     = LENH Pi gui xuong. Bong ma chay truoc ma robot dac dung yen la firmware dang bo lenh
  ban, vat    = theo worlds/pick_place.sdf, vat dinh vao kep khi kep dong
  dong chu    = trang thai STM32 va sai lech lenh - thuc tung khop

Thu tu khoi dong:
  1. robot_state_publisher, (stm32_emulator), stm32_bridge, ros2_control_node, (twin)
  2. Sau 3 s (cho trang thai khop tu STM32 ve on dinh): joint_state_broadcaster
  3. Sau do: arm_controller, gripper_controller
  4. Sau khi arm_controller chay: move_group (va RViz neu use_rviz:=true)
"""

from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler,
                            TimerAction)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


PACKAGE_NAME = 'robot_arm_4_dof'
MOVEIT_PACKAGE = 'moveit_robot_arm'
ROBOT_NAME = 'robot_arm_4_dof'
EMULATOR_LINK = '/tmp/ttySTM32'

# Gioi han firmware gia dinh khi relax_limits:=true (do, theo quy uoc firmware)
RELAXED_MIN_DEG = [-90.0, -60.0, -90.0, -45.0]


def launch_setup(context, *args, **kwargs):
    hw = LaunchConfiguration('hw').perform(context).lower()
    if hw not in ('real', 'emulator', 'loopback'):
        raise RuntimeError(f'hw:={hw} khong hop le. Chon real, emulator hoac loopback')
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'
    twin = LaunchConfiguration('twin').perform(context).lower() == 'true'
    relax = LaunchConfiguration('relax_limits').perform(context).lower() == 'true'
    port = LaunchConfiguration('port').perform(context)

    package_share = Path(get_package_share_directory(PACKAGE_NAME))
    moveit_share = Path(get_package_share_directory(MOVEIT_PACKAGE))
    xacro_file = package_share / 'urdf' / 'robot_arm_4_dof.urdf.xacro'
    controllers_file = package_share / 'config' / 'final.yaml'
    moveit_controllers_file = package_share / 'config' / 'moveit_controllers_gazebo.yaml'
    bridge_config = package_share / 'config' / 'stm32_bridge.yaml'
    fw_model_config = package_share / 'config' / 'stm32_firmware_model.yaml'
    rviz_file = (package_share / 'config' / 'twin.rviz') if twin else (moveit_share / 'config' / 'moveit.rviz')

    # URDF voi khoi ros2_control cho robot that (JointStateTopicSystem)
    robot_description = xacro.process_file(
        str(xacro_file), mappings={'use_real_hw': 'true'}
    ).toxml()

    moveit_config = (
        MoveItConfigsBuilder(ROBOT_NAME, package_name=MOVEIT_PACKAGE)
        .robot_description(file_path=str(xacro_file), mappings={'use_real_hw': 'true'})
        .trajectory_execution(file_path=str(moveit_controllers_file))
        .to_moveit_configs()
    )

    actions = [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        output='screen',
    )]

    # ---------------- STM32: that / gia lap ----------------
    if hw == 'emulator':
        overrides = {'link_path': EMULATOR_LINK}
        if relax:
            overrides['min_deg'] = RELAXED_MIN_DEG
        actions.append(Node(
            package=PACKAGE_NAME,
            executable='stm32_emulator_node.py',
            name='stm32_emulator',
            parameters=[str(fw_model_config), overrides],
            output='screen',
        ))
        port = EMULATOR_LINK

    actions.append(Node(
        package=PACKAGE_NAME,
        executable='stm32_bridge_node.py',
        name='stm32_bridge',
        parameters=[str(bridge_config), {
            'port': port,
            'loopback': hw == 'loopback',
            # Robot that: KHONG tu bat, phai goi ~/enable bang tay
            'auto_enable': hw != 'real',
        }],
        output='screen',
    ))

    # Jazzy: controller_manager tu nhan URDF qua topic /robot_description
    actions.append(Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[str(controllers_file)],
        output='screen',
    ))

    # ---------------- bong ma lenh + ban, vat, sai lech ----------------
    if twin:
        actions += [
            Node(
                package=PACKAGE_NAME,
                executable='twin_monitor_node.py',
                name='twin_monitor',
                output='screen',
            ),
            # robot_state_publisher thu 2: ve tu the LENH, khung TF co tien to cmd/
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='cmd_state_publisher',
                parameters=[{'robot_description': robot_description, 'frame_prefix': 'cmd/'}],
                remappings=[('joint_states', '/twin/joint_commands'),
                            ('robot_description', '/twin/cmd_robot_description')],
                output='log',
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='cmd_world_tf',
                arguments=['--frame-id', 'world', '--child-frame-id', 'cmd/world'],
                output='log',
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
    arm = spawner('arm_controller')
    gripper = spawner('gripper_controller')

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[moveit_config.to_dict(),
                    {'use_sim_time': False, 'publish_robot_description_semantic': True}],
        output='screen',
    )

    after_arm = [move_group]
    if use_rviz:
        after_arm.append(Node(
            package='rviz2', executable='rviz2', arguments=['-d', str(rviz_file)],
            parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic,
                        moveit_config.robot_description_kinematics, moveit_config.planning_pipelines,
                        moveit_config.joint_limits, {'use_sim_time': False}],
            output='screen',
        ))

    actions += [
        # Cho trang thai that ve truoc khi bat controller, tranh controller giu vi tri 0
        TimerAction(period=3.0, actions=[jsb]),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm, gripper])),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=after_arm)),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hw', default_value='real',
                              description='real: robot that | emulator: STM32 gia lap de xem truoc | '
                                          'loopback: gia lap tho'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                              description='Cong noi tiep toi USART1 cua STM32 (chi dung khi hw:=real)'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Mo RViz tren may nay (Pi khong co man hinh thi de false)'),
        DeclareLaunchArgument('twin', default_value='true',
                              description='Ve bong ma lenh, ban, vat va theo doi sai lech lenh - thuc'),
        DeclareLaunchArgument('relax_limits', default_value='false',
                              description='Chi voi hw:=emulator: gia dinh gioi han firmware J2 >= -60, '
                                          'J4 >= -45 do'),
        OpaqueFunction(function=launch_setup),
    ])
