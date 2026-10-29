import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # declare the launch argument
    model = DeclareLaunchArgument(
        "model",
        default_value="emrox_ur30_d435_epick",
        description="URDF/XACRO model name (without file extension)",
    )

    joint_positions_file = os.path.join(
        get_package_share_directory("container_unloading_sim_benchmark"),
        "config",
        "ur30",
        "initial_positions.yaml",
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("container_unloading_sim_benchmark"),
                    "launch",
                    "robot_description_bringup.launch.py",
                )
            ]
        ),
        launch_arguments={
            "model": LaunchConfiguration("model"),
        }.items(),
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[joint_positions_file, {"use_sim_time": False}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("container_unloading_sim_benchmark"),
                "config",
                "view_config.rviz",
            ),
        ],
        parameters=[{"use_sim_time": False}],
        on_exit=Shutdown(),
    )

    return LaunchDescription(
        [model, robot_description_launch, joint_state_publisher_gui, rviz_node]
    )
