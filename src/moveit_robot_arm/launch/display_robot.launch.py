
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # 1. Lay duong dan package robot
    package_name = "robot_arm_4_dof"
    package_dir = Path(get_package_share_directory(package_name))

    # 2. Duong dan file Xacro
    xacro_file = (
        package_dir / "urdf" / "robot_arm_4_dof.urdf.xacro"
    )

    if not xacro_file.is_file():
        raise FileNotFoundError(
            f"Khong tim thay file Xacro: {xacro_file}"
        )

    # 3. Tu dong xu ly Xacro -> URDF
    robot_description = xacro.process_file(
        str(xacro_file)
    ).toxml()

    # Chuyen model:// sang package:// cho RViz
    robot_description = robot_description.replace(
        "model://robot_arm_4_dof/meshes/",
        "package://robot_arm_4_dof/meshes/"
    )

    robot_description_param = {
        "robot_description": robot_description
    }

    # 4. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description_param],
        output="screen"
    )

    # 5. GUI dieu chinh goc cac khop
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[robot_description_param],
        output="screen"
    )

    # 6. Cau hinh RViz (neu da co)
    rviz_config = package_dir / "config" / "display.rviz"

    rviz_args = (
        ["-d", str(rviz_config)]
        if rviz_config.is_file()
        else []
    )

    # 7. Khoi dong RViz 2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=rviz_args,
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])
